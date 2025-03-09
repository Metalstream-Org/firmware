#include <stdio.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <esp_adc/adc_oneshot.h>
#include "string.h"
#include "sensor.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include <vector>
#include <string>
#include <queue>
#include "config.h"
#include <algorithm>
#include "sensor.h"

#include "sampler_task.h"
#include "serial_task.h"

static const char* TAG = "Main";


// TODO! better naming
class MeasuremenstState
{
    public:
        MeasuremenstState()
        {

        }

        void update(int64_t timestamp, int width)
        {
            m_max_width = std::max<int>(width, m_max_width);

            ESP_LOGI(TAG, "width is: %d and max width is: %d", width, m_max_width);

            if (m_start_timestamp == 0)
            {
                m_start_timestamp = timestamp;
            }
        }

        void clear()
        {
            m_max_width = 0;
            m_start_timestamp = 0;
        }

        int get_max_width() const { return m_max_width; };
        int64_t get_start_timestamp() const { return m_start_timestamp; }

    private:
        // the maximum measured width
        int m_max_width;
        int64_t m_start_timestamp = 0;
};

// 4 bytes because command consists of 3 characters + zero terminator string
void send_message(const QueueHandle_t queue, const char command[4], const char* payload) // |\label{line:send_message_function}|
{
    // ESP_LOGI(TAG, "Send message\n");
    static char buf[MAX_MESSAGE_LEN];

    memset(buf, 0x00, sizeof(buf));

    auto timestamp = esp_timer_get_time() / 1000;
    snprintf(buf, sizeof(buf), "$%lld:%s:%s#\r\n", timestamp, command, payload);
    
    xQueueSend(queue, buf, portMAX_DELAY);
}


void send_heartbeat(const QueueHandle_t queue)
{
    static int sequence = 0;

    sequence++;

    char payload[12];

    snprintf(payload, sizeof(payload), "%d", sequence);
    send_message(queue, "HBT", payload);
}

void send_sensor_measurements(const QueueHandle_t queue, const SamplerQueueItem& item) // |\label{line:send_sensor_measurement_function}|
{
    static char payload[MAX_MESSAGE_LEN];

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        memset(payload, 0x00, sizeof(payload));

        auto sensor = item.results[i];
        snprintf(payload, sizeof(payload), "T=%lld:ID=%d:C=%d:V=%d",
            item.timestamp, 
            sensor.id,
            sensor.connected,
            sensor.value);
        
        send_message(queue, "SMS", payload);
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Metalshare Hub...");

    // Configureer USB CDC driver.
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = USB_FIFO_BUF_SIZE,
        .rx_buffer_size = USB_FIFO_BUF_SIZE,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));

    // Initialiseer en configureer ADC driver. Alle analoge ingangen zitten op ADC1.
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc1_handle));

    // Initialiseer en configureer sensoren.
    Sensor sensors[NUM_SENSORS] = {
        Sensor(1, 1500, adc1_handle, ADC_CHANNEL_6, GPIO_NUM_18, true, 1100),
        Sensor(2, 1500, adc1_handle, ADC_CHANNEL_5, GPIO_NUM_17, false, 2100),
        Sensor(3, 1500, adc1_handle, ADC_CHANNEL_4, GPIO_NUM_16, true, 3300),
        Sensor(4, 1500, adc1_handle, ADC_CHANNEL_0, GPIO_NUM_15, false, 4350),
        Sensor(5, 1500, adc1_handle, ADC_CHANNEL_7, GPIO_NUM_21, false, 300),
        Sensor(6, 1500, adc1_handle, ADC_CHANNEL_9, GPIO_NUM_47, false, 360),
        Sensor(7, 1500, adc1_handle, ADC_CHANNEL_8, GPIO_NUM_48, false, 420),
        Sensor(8, 1500, adc1_handle, ADC_CHANNEL_2, GPIO_NUM_45, false, 480)
    };

    char rx_buf[MAX_MESSAGE_LEN];

    // Initialiseer and configureer sampler_task
    auto queue = xQueueCreate(5, sizeof(SamplerQueueItem));
    SamplerTaskParams sampler_params = {queue, NUM_SENSORS, sensors};
    xTaskCreate(sampler_task, "sampler", 2048, &sampler_params, 1, nullptr);

    // Initialiseer and configureer serial_task
    auto serial_tx_queue = xQueueCreate(10, sizeof(char[MAX_MESSAGE_LEN]));
    auto serial_rx_queue = xQueueCreate(10, sizeof(char[MAX_MESSAGE_LEN]));

    SerialTaskParams serial_params = {.tx_queue=serial_tx_queue, .rx_queue=serial_rx_queue};
    xTaskCreate(serial_task, "serial", 8192, &serial_params, 1, nullptr);

    SamplerQueueItem sampler_queue_item;
    // De delay buffer wordt gebruikt om samples te vertragen, zodat we de tijd tussen de bovenste en onderste rij kunnen berekenen.
    std::queue<SamplerQueueItem> delay_buffer;

    // Snelheid van de band in cm/s, wordt geinitialiseerd met de standaard snelheid, maar kan worden gecalibreerd.
    float conveyer_belt_speed = INITIAL_CONVEYER_BELT_SPEED;

    // De kalibratie tijd, is de tijd die tussen de bovenste en onderste rij zit. Er geldt s=v*t. Hierbij is s, de longitudinale afstand tussen de sensoren en v, de snelheid van de band. 
    size_t calibration_time_ms = (SENSOR_LONGITUDINAL_SAPCING*100)/conveyer_belt_speed;
    MeasuremenstState measurements_state;

    int64_t calibration_start_timestamp = 0;
    bool calibrated = false;

    while(true)
    {
        // Bereken het aantal samples dat wordt genomen in de tijd tussen de bovenste en onderste rij.
        size_t num_delayed_samples = calibration_time_ms * SAMPLE_RATE / 1000;

        // Hier wordt gebruik gemaakt van een while loop, omdat we alle inkomende samples willen verwerken, als we dit niet doen loopt de queue over.
        while (xQueueReceive(queue, &sampler_queue_item, pdMS_TO_TICKS(1)))
        {
            // Voeg elk sample wat binnenkomt ook toe aan de delay buffer, hierdoor kunnen we later dynamisch nog de plaatsing van de sensoren veranderen.
            delay_buffer.push(sampler_queue_item); // |\label{line:add_to_delay_buffer}|

            // De samples op de tweede rij moeten met de tijd tussen rij een en rij twee worden vertraagd, dit is op basis van de snelheid van de lopende band.
            // Vervolgens rekenen we met die tijd en de sample rate het aantal samples dat in deze tijd genomen worden.
            // Op basis hiervan bepalen we de delay buffer grootte. We checken hier of de grootte van de delay buffer groter is, omdat hiervoor al een sample wordt gepushed naar de delay buffer
            if (delay_buffer.size() > num_delayed_samples)
            {
                // De delay buffer werkt als een FIFO buffer. We willen hier dus het eerste item uit de queue halen.
                // Deze is vertraagd met het aantal samples dat in de calibration_time_ms kan worden genomen, oftwel de tijd tussen de bovenste en onderste rij.
                auto delayed_queue_item = delay_buffer.front(); // |\label{line:delayed_sample}|
                delay_buffer.pop();

                const SensorResult* leftmost_sensor = nullptr;
                const SensorResult* rightmost_sensor = nullptr;

                for (size_t i = 0; i < NUM_SENSORS; i++)
                {
                    const SensorResult& result = sensors[i].is_delayed() 
                        ? delayed_queue_item.results[i] 
                        : sampler_queue_item.results[i];


                    if (result.connected && sensors[i].is_value_above_threshold(result.value))
                    {
                        ESP_LOGI(TAG, "Sensor %02d detected metal, %d", result.id, result.value);
                        if (!leftmost_sensor || sensors[i].get_x() < sensors[leftmost_sensor->id - 1].get_x()) {
                            leftmost_sensor = &result;
                        }

                        if (!rightmost_sensor || sensors[i].get_x() > sensors[rightmost_sensor->id - 1].get_x()) {
                            rightmost_sensor = &result;
                        }
                    }
                }

                // Bereken breedte als beide sensoren zijn gevonden
                if (leftmost_sensor && rightmost_sensor)
                {
                    int width = sensors[rightmost_sensor->id - 1].get_x() - sensors[leftmost_sensor->id - 1].get_x() + SENSOR_WIDTH * (rightmost_sensor->id-leftmost_sensor->id); // |\label{line:determine_width}|
                    ESP_LOGI(TAG, "Breedte: %d mm (sensor %d tot %d) value: %d",
                        width, leftmost_sensor->id, rightmost_sensor->id, sampler_queue_item.results[0].value);

                    measurements_state.update(sampler_queue_item.timestamp, width);
                } else
                {
                    auto width = measurements_state.get_max_width();
                    auto timestamp = measurements_state.get_start_timestamp();

                    if (timestamp > 0) {
                        // TODO!: Config check for [0] sensor
                        // Delta tijd in seconden
                        float delta_in_s =  (sampler_queue_item.timestamp - timestamp) / 1000.0 / 1000.0;
                        
                        uint16_t length = static_cast<uint16_t>((conveyer_belt_speed*10)*delta_in_s);

                        char payload[MAX_MESSAGE_LEN];

                        snprintf(payload, sizeof(payload), "W=%d:L=%d:S=%f",
                                    width, length, conveyer_belt_speed);

                        send_message(serial_tx_queue, "MET", payload);


                        ESP_LOGI(TAG, "Got result - timestamp: %5.2fs, length: %dmm, width: %dmm, speed: %fcm/s", delta_in_s, length, width, conveyer_belt_speed);
                    }

                    measurements_state.clear();
                }
            }

   
            send_sensor_measurements(serial_tx_queue, sampler_queue_item);
        }

        if (xQueueReceive(serial_rx_queue, &rx_buf, pdMS_TO_TICKS(1)))
        {
            char *token;   
            token = strtok(rx_buf, ":");

            while(token != nullptr) {
                if (strcmp(token, "CAL") == 0) {
                    ESP_LOGI(TAG, "CALIBRATION MODE met start time: %lld en calibration time: %d\n", calibration_start_timestamp, calibration_time_ms);

                    calibrated = false;
                    calibration_start_timestamp = 0;

                    while(!calibrated)
                    {
                        while (xQueueReceive(queue, &sampler_queue_item, pdMS_TO_TICKS(1)))
                        {
                            for (size_t i = 0; i < NUM_SENSORS; i++)
                            {
                                if (sampler_queue_item.results[i].connected && sensors[i].is_value_above_threshold(sampler_queue_item.results[i].value))
                                {
                                    if (sensors[i].is_delayed() && calibration_start_timestamp == 0)
                                    {
                                        calibration_start_timestamp = sampler_queue_item.timestamp;
                                        ESP_LOGI(TAG, "First calibration timestamp: %lld", calibration_start_timestamp);
                                    } else if (!sensors[i].is_delayed() && calibration_start_timestamp != 0)
                                    {
                                        calibration_time_ms = (sampler_queue_item.timestamp - calibration_start_timestamp) / 1000;
                                        // s = v*t -> v=s/t where s is equal to the distance between the front and back row and t is equal to the calibration time in seconds.

                                        if (calibration_time_ms > 0)
                                        {
                                            conveyer_belt_speed = (SENSOR_LONGITUDINAL_SAPCING/10.0)/(calibration_time_ms/1000.0);
                                            ESP_LOGI(TAG, "Calibrated, the conveyer belt speed is: %fcm/s, the calibration time is: %dms", conveyer_belt_speed, calibration_time_ms);
                                        } else
                                        {
                                            ESP_LOGI(TAG, "Calibration time equal to zero, calibration aborted, fallback to: %fcm/s", conveyer_belt_speed);
                                        }
                                        
                                        calibration_start_timestamp = 0;
                                        calibrated = true;
                                    
                                    // Abort if calibration takes too long, fallback to default speed
                                    } else if (sensors[i].is_delayed() && calibration_start_timestamp != 0 )
                                    {
                                        int delta_time = sampler_queue_item.timestamp - calibration_start_timestamp;

                                        if (delta_time > MAX_CALIBRATION_TIME_MS*1000)
                                        {
                                            ESP_LOGI(TAG, "Calibration took to look");
                                            calibrated = true;
                                        }
                                    }
                                }
                            }
                        }

                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                } else {
                    ESP_LOGI(TAG, "Unknown token: %s\n", token);
                }

                token = strtok(nullptr, ":");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));

    }
}