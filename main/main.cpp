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
void send_message(const QueueHandle_t queue, const char command[4], const char* payload)
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

void send_sensor_measurements(const QueueHandle_t queue, const SamplerQueueItem& item)
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
        
        // ESP_LOGI(TAG, "Sending message for sensor: %d", i+1);
        send_message(queue, "SMS", payload);
    }

    // TODO: naam
}

void send_dimensions(const QueueHandle_t queue, const SamplerQueueItem& item)
{
    static char payload[MAX_MESSAGE_LEN];
    memset(payload, 0x00, sizeof(payload));

    int offset = 0;

    offset += snprintf(&payload[offset], sizeof(payload) - offset, "T=%lld:", item.timestamp);

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        const char* sep = (i==(NUM_SENSORS-1)) ? "" : ":"; 
        
        auto sensor = item.results[i];
        offset += snprintf(&payload[offset], sizeof(payload) - offset, "ID=%d:C=%d:V=%d%s", 
            sensor.id,
            sensor.connected,
            sensor.value,
            sep);
    }

    // TODO: naam
    send_message(queue, "SMS", payload);
}

extern "C" void app_main(void)
{
    // Initialiseer seriële communicatie via usb cdc poort
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = BUF_SIZE,
        .rx_buffer_size = BUF_SIZE,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));

    // Initialiseer ADC driver
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,  // Gebruik ADC Unit 1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc1_handle));

    Sensor sensors[NUM_SENSORS] = {
        Sensor(1, 1300, adc1_handle, ADC_CHANNEL_5, GPIO_NUM_41, true, 0),
        Sensor(2, 1300, adc1_handle, ADC_CHANNEL_4, GPIO_NUM_42, false, 800),
        Sensor(3, 1300, adc1_handle, ADC_CHANNEL_3, GPIO_NUM_2, true, 1600),
        Sensor(4, 1300, adc1_handle, ADC_CHANNEL_6, GPIO_NUM_1, false, 2400),
        Sensor(5, 1300, adc1_handle, ADC_CHANNEL_7, GPIO_NUM_21, false, 300),
        Sensor(6, 1300, adc1_handle, ADC_CHANNEL_9, GPIO_NUM_47, true, 360),
        Sensor(7, 1300, adc1_handle, ADC_CHANNEL_8, GPIO_NUM_48, false, 420),
        Sensor(8, 1300, adc1_handle, ADC_CHANNEL_2, GPIO_NUM_45, true, 480)
    };

    auto queue = xQueueCreate(5, sizeof(SamplerQueueItem));
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

    std::queue<SamplerQueueItem> delay_buffer;


    // Conveyer belt speed in cm/s
    float conveyer_belt_speed = 9.4;

    // the time between top and bottom sensors, should be calibrated
    // s=v*t where s=10cm (distance between sensors front/back) and v=9.4cm (conveyer belt speed). 
    size_t calibration_time_ms = (10*1000)/conveyer_belt_speed;
    MeasuremenstState measurements_state;

    int64_t calibration_start_timestamp = 0;
    bool calibrated = false;

    ESP_LOGI(TAG, "Starting...\n");

    while(true)
    {
        size_t num_delayed_samples = calibration_time_ms * SAMPLE_RATE / 1000;

        // Hier wordt gebruik gemaakt van een while loop, omdat we alle inkomende samples willen verwerken, als we dit niet doen loopt de queue over
        while (xQueueReceive(queue, &sampler_queue_item, pdMS_TO_TICKS(1)))
        {
            // Voeg elk sample wat binnenkomt ook toe aan de delay buffer, hierdoor kunnen we later dynamisch nog de plaatsing van de sensoren veranderen
            delay_buffer.push(sampler_queue_item);

            // De samples op de tweede rij moeten met de tijd tussen rij een en rij twee worden vertraagd, dit is op basis van de snelheid van de lopende band. Vervolgens rekenen we met die tijd en de sample rate het aantal samples dat in deze tijd genomen worden. Op basis hiervan bepalen we de delay buffer grootte. We checken hier of de grootte van de delay buffer groter is, omdat hiervoor al een sample wordt gepushed naar de delay buffer
            if (delay_buffer.size() > num_delayed_samples)
            {
                // De delay buffer werkt als een FIFO buffer. We willen hier dus het eerste item uit de queue halen. Deze is vertraagd met de tijd tussen de rijen
                auto delayed_queue_item = delay_buffer.front();
                delay_buffer.pop();

                // ESP_LOGI(TAG, "delay sample time: %lld, sample time: %lld, delta: %lld", delayed_queue_item.timestamp, sampler_queue_item.timestamp, sampler_queue_item.timestamp-delayed_queue_item.timestamp);

                // printf("item timestamp: %lld - delayed item timestamp: %lld | DELTA: %lld\n", sampler_queue_item.timestamp, delayed_queue_item.timestamp, (sampler_queue_item.timestamp - delayed_queue_item.timestamp)/1000);

                const SensorResult* leftmost_sensor = nullptr;
                const SensorResult* rightmost_sensor = nullptr;

                for (size_t i = 0; i < NUM_SENSORS; i++)
                {
                    const SensorResult& result = sensors[i].is_delayed() 
                        ? delayed_queue_item.results[i] 
                        : sampler_queue_item.results[i];


                    if (result.connected && sensors[i].is_above_threshold(result.value))
                    {
                        // ESP_LOGI(TAG, "Sensor S0%d detected metal, %d", result.id, result.value);
                        // printf("sensor id: %d, detected value: %d\n", result.id, result.value);
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
                    int width = sensors[rightmost_sensor->id - 1].get_x() - sensors[leftmost_sensor->id - 1].get_x() + SENSOR_WIDTH;
                    ESP_LOGI(TAG, "Breedte: %d mm (sensor %d tot %d)",
                        width, leftmost_sensor->id, rightmost_sensor->id);

                    measurements_state.update(sampler_queue_item.timestamp, width);
                } else
                {
                    auto width = measurements_state.get_max_width();
                    auto timestamp = measurements_state.get_start_timestamp();

                    if (timestamp > 0) {
                        // TODO!: Config check for [0] sensor
                        // Delta tijd in seconden
                        float delta_in_s =  (sampler_queue_item.timestamp - timestamp) / 1000.0 / 1000.0;
                        
                        // s=v*t
                        size_t length = static_cast<size_t>((conveyer_belt_speed*10)*delta_in_s);

                        char payload[MAX_MESSAGE_LEN];

                        snprintf(payload, sizeof(payload), "W=%d:L=%i",
                                    width, length);

                        send_message(serial_tx_queue, "MET", payload);


                        ESP_LOGI(TAG, "Got result - timestamp: %5.2fs, length: %dmm, width: %dmm", delta_in_s, length, width);
                    }

                    measurements_state.clear();
                }


                // printf("first item timestamp: %lld - second item timestamp: %lld | DELTA: %lld\n", combined_results[0].timestamp, combined_results[1].timestamp, (combined_results[0].timestamp - combined_results[1].timestamp)/1000);

            }

   
            // ESP_LOGI(TAG, "sending measurements at %lld. SENSOR 1 connected: %d", sampler_queue_item.timestamp, sampler_queue_item.results[0].connected);
            send_sensor_measurements(serial_tx_queue, sampler_queue_item);
                // printf("S0%d - timestamp: %lld, connected: %d, sampled: %d\n", i+1, sampler_queue_item.results[i].timestamp, sampler_queue_item.results[i].connected, sampler_queue_item.results[i].value);        
        }

        if (xQueueReceive(serial_rx_queue, &rx_buf, pdMS_TO_TICKS(1)))
        {
            ESP_LOGD(TAG, "Queue receive RX: %s\n", rx_buf);

            char *token;
            
            /* get the first token */
            token = strtok(rx_buf, ":");

            ESP_LOGI(TAG, "%s", token);


            while(token != nullptr) {
                if (strcmp(token, "CAL") == 0) {
                    ESP_LOGI(TAG, "CALIBRATION MODE\n");

                    calibrated = false;
                    calibration_start_timestamp = 0;

                    while(!calibrated)
                    {
                        while (xQueueReceive(queue, &sampler_queue_item, pdMS_TO_TICKS(1)))
                        {
                            // ESP_LOGI(TAG, "timestamp sample: %lld", sampler_queue_item.timestamp);

                            for (size_t i = 0; i < NUM_SENSORS; i++)
                            {
                                if (sampler_queue_item.results[i].connected && sensors[i].is_above_threshold(sampler_queue_item.results[i].value))
                                {
                                    if (sensors[i].is_delayed() && calibration_start_timestamp == 0)
                                    {
                                        calibration_start_timestamp = sampler_queue_item.timestamp;
                                        ESP_LOGI(TAG, "First calibration timestamp: %lld", calibration_start_timestamp);
                                    }
                                    
                                    // TODO!
                                    
                                    if (!sensors[i].is_delayed() && calibration_start_timestamp != 0) {
                                        calibration_time_ms = (sampler_queue_item.timestamp - calibration_start_timestamp) / 1000;
                                        // s = v*t -> v=s/t where s is equal to the distance between the front and back row and t is equal to the calibration time in seconds.
                                        conveyer_belt_speed = (SENSOR_LONGITUDINAL_SAPCING/10.0)/(calibration_time_ms/1000.0);
                                        calibration_start_timestamp = 0;

                                        ESP_LOGI(TAG, "Calibrated, the conveyer belt speed is: %fcm/s, the calibration time is: %dms", conveyer_belt_speed, calibration_time_ms);
                                        calibrated = true;
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