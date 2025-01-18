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

const size_t BUF_SIZE = 256;
const size_t ECHO_TASK_STACK_SIZE = 4096;
const size_t MAX_MESSAGE_LEN = 256;
const size_t SAMPLE_RATE = 10;
const size_t SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE;

struct SerialTaskParams
{
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
};


struct SensorResult
{
    size_t id;
    bool connected;
    int value;
};

struct SamplerQueueItem
{
    int64_t timestamp;
    // TODO! remove timstamp from sensor result
    SensorResult results[NUM_SENSORS];
};

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

struct SamplerTaskParams
{
    QueueHandle_t queue;
    size_t num_sensors;
    Sensor* sensors;
};


// Samples sensors in seperate task 
void sampler(void* parameters)
{
    auto params = static_cast<SamplerTaskParams*>(parameters);

    SamplerQueueItem queue_item;

    while (1) {
        memset(&queue_item, 0, sizeof(queue_item));
        queue_item.timestamp = esp_timer_get_time();

        // Sample the sensors
        for (int i = 0; i < params->num_sensors; i++)
        {
            queue_item.results[i].id = i + 1;
            queue_item.results[i].connected = params->sensors[i].is_connected();
            queue_item.results[i].value = params->sensors[i].sample();
        }

        // Verzend het sample naar de queue
        xQueueSend(params->queue, &queue_item, portMAX_DELAY);
        // Hier wordt gebruik gemaakt van een delay om te zorgen dat de task sampled op de juiste frequentie (SAMPLE_INTERVAL)
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}


void serial_task(void* parameters)
{
    auto params = static_cast<SerialTaskParams*>(parameters);

    char tx_buf[MAX_MESSAGE_LEN];
    memset(tx_buf, 0, sizeof(tx_buf));
    char rx_buf[MAX_MESSAGE_LEN];
    memset(rx_buf, 0, sizeof(rx_buf));

    size_t rd_len = 0;
    char temp_rx_buf[64];

    while (true)
    {
        if (xQueueReceive(params->tx_queue, tx_buf, portMAX_DELAY)) {
            usb_serial_jtag_write_bytes(tx_buf, strlen(tx_buf), 20 / portTICK_PERIOD_MS);
            memset(tx_buf, 0, sizeof(tx_buf));
        }

        int len = usb_serial_jtag_read_bytes(temp_rx_buf, sizeof(temp_rx_buf), 20 / portTICK_PERIOD_MS);

        // Write data back to the USB SERIAL JTAG
        if (len > 0) {

            // check if we have enough space in rd_buf
            if ((rd_len + len + 1)  < sizeof (rx_buf)) {
                // copy in new received data
                memcpy(&rx_buf[rd_len], temp_rx_buf, len);
                memset(temp_rx_buf, 0, sizeof(temp_rx_buf));
                rd_len += len;
            } else {
                // discard everything when buffer overflows
                rd_len = 0;
            }

            // make sure string is terminated
            rx_buf[rd_len] = '\0';

            // parse message
            if (rd_len > 0 ) {
                char* start = strchr((const char*)rx_buf, '$');
                
                // remove everything before start
                if (start != nullptr)
                {
                    rd_len = rd_len - ((size_t)start - (size_t)rx_buf);
                    memmove(rx_buf, start, rd_len + 1);

                    // check if we have a complete message
                    char* end = strchr((const char*)rx_buf, '#');
                    
                    if (end != nullptr) {
                        // construct output message (in data), skip for character ($)
                        size_t olen = (size_t)end - (size_t)rx_buf;
                        rx_buf[olen] = '\0';

                        xQueueSend(params->rx_queue, &rx_buf[1], portMAX_DELAY);
                        
                        rd_len = 0;
                        memset(rx_buf, 0x00, sizeof(rx_buf));
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}



// 4 bytes because command consists of 3 characters + zero terminator string
void send_message(const QueueHandle_t queue, const char command[4], const char* payload)
{
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
        Sensor(1, 1300, adc1_handle, ADC_CHANNEL_5, GPIO_NUM_41, false, 60, 0),
        Sensor(2, 1300, adc1_handle, ADC_CHANNEL_4, GPIO_NUM_42, true, 120, 50),
        Sensor(3, 1300, adc1_handle, ADC_CHANNEL_3, GPIO_NUM_2, false, 180, 0),
        Sensor(4, 1300, adc1_handle, ADC_CHANNEL_6, GPIO_NUM_1, true, 240, 0),
        Sensor(5, 1300, adc1_handle, ADC_CHANNEL_7, GPIO_NUM_21, false, 300, 0),
        Sensor(6, 1300, adc1_handle, ADC_CHANNEL_9, GPIO_NUM_47, true, 360, 0),
        Sensor(7, 1300, adc1_handle, ADC_CHANNEL_8, GPIO_NUM_48, false, 420, 0),
        Sensor(8, 1300, adc1_handle, ADC_CHANNEL_2, GPIO_NUM_45, true, 480, 0)
    };

    auto queue = xQueueCreate(5, sizeof(SamplerQueueItem));
    char rx_buf[MAX_MESSAGE_LEN];

    // Create sampler task
    SamplerTaskParams sampler_params = {queue, NUM_SENSORS, sensors};
    xTaskCreate(sampler, "sampler", 2048, &sampler_params, 5, nullptr);

    // Create serial task
    auto serial_tx_queue = xQueueCreate(50, sizeof(char[MAX_MESSAGE_LEN]));
    auto serial_rx_queue = xQueueCreate(10, sizeof(char[MAX_MESSAGE_LEN]));

    SerialTaskParams serial_params = {.tx_queue=serial_tx_queue, .rx_queue=serial_rx_queue};
    xTaskCreate(serial_task, "serial", 8192, &serial_params, 5, nullptr);

    SamplerQueueItem sampler_queue_item;

    std::queue<SamplerQueueItem> delay_buffer;

    // the time between top and bottom sensors, should be calibrated
    size_t calibration_time_ms = 2100;
    size_t num_delayed_samples = calibration_time_ms * SAMPLE_RATE / 1000;

    MeasuremenstState measurements_state;

    while(true)
    {
        // Hier wordt gebruik gemaakt van een while loop, omdat we alle inkomende samples willen verwerken, als we dit niet doen loopt de queue over
        while (xQueueReceive(queue, &sampler_queue_item, pdMS_TO_TICKS(1))) {
            // Voeg elk sample wat binnenkomt ook toe aan de delay buffer, hierdoor kunnen we later dynamisch nog de plaatsing van de sensoren veranderen
            delay_buffer.push(sampler_queue_item);

            // De samples op de tweede rij moeten met de tijd tussen rij een en rij twee worden vertraagd, dit is op basis van de snelheid van de lopende band. Vervolgens rekenen we met die tijd en de sample rate het aantal samples dat in deze tijd genomen worden. Op basis hiervan bepalen we de delay buffer grootte. We checken hier of de grootte van de delay buffer groter is, omdat hiervoor al een sample wordt gepushed naar de delay buffer
            if (delay_buffer.size() > num_delayed_samples)
            {
                // De delay buffer werkt als een FIFO buffer. We willen hier dus het eerste item uit de queue halen. Deze is vertraagd met de tijd tussen de rijen
                auto delayed_queue_item = delay_buffer.front();
                delay_buffer.pop();

                // printf("item timestamp: %lld - delayed item timestamp: %lld | DELTA: %lld\n", sampler_queue_item.timestamp, delayed_queue_item.timestamp, (sampler_queue_item.timestamp - delayed_queue_item.timestamp)/1000);

                const SensorResult* left_sensor = nullptr;
                const SensorResult* right_sensor = nullptr;

                for (size_t i = 0; i < NUM_SENSORS; i++)
                {
                    const SensorResult& result = sensors[i].is_delayed() 
                        ? delayed_queue_item.results[i] 
                        : sampler_queue_item.results[i];

                    if (result.connected && result.value >= sensors[i].get_threshold())
                    {
                        // printf("sensor id: %d, detected value: %d\n", result.id, result.value);
                        if (!left_sensor || sensors[i].get_x() < sensors[left_sensor->id - 1].get_x()) {
                            left_sensor = &result;
                        }

                        if (!right_sensor || sensors[i].get_x() > sensors[right_sensor->id - 1].get_x()) {
                            right_sensor = &result;
                        }
                    }
                }

                // Bereken breedte als beide sensoren zijn gevonden
                if (left_sensor && right_sensor)
                {
                    int width = sensors[right_sensor->id - 1].get_x() - sensors[left_sensor->id - 1].get_x() + SENSOR_WIDTH;
                    printf("Breedte: %d mm (sensor %d tot %d)\n",
                        width, left_sensor->id, right_sensor->id);

                    measurements_state.update(sampler_queue_item.timestamp, width);
                } else
                {
                    auto width = measurements_state.get_max_width();
                    auto timestamp = measurements_state.get_start_timestamp();

                    if (timestamp > 0) {
                        // TODO!: Config check for [0] sensor
                        int64_t delta =  sampler_queue_item.timestamp - timestamp;

                        printf("Got result - timestamp: %lld, width: %d\n", delta, width);
                    }

                    measurements_state.clear();
                }


                // printf("first item timestamp: %lld - second item timestamp: %lld | DELTA: %lld\n", combined_results[0].timestamp, combined_results[1].timestamp, (combined_results[0].timestamp - combined_results[1].timestamp)/1000);

            }

   
            send_sensor_measurements(serial_tx_queue, sampler_queue_item);
                // printf("S0%d - timestamp: %lld, connected: %d, sampled: %d\n", i+1, sampler_queue_item.results[i].timestamp, sampler_queue_item.results[i].connected, sampler_queue_item.results[i].value);        
        }

        if (xQueueReceive(serial_rx_queue, &rx_buf, pdMS_TO_TICKS(1)))
        {
            // printf("Queue receive RX: %s\n", rx_buf);

            // char *token;
            
            // /* get the first token */
            // token = strtok(rx_buf, ":");

            // while(token != nullptr) {
            //     printf("token: '%s' \n", token);
            //     token = strtok(nullptr, ":");
            // }
        }

        vTaskDelay(pdMS_TO_TICKS(10));

    }
}