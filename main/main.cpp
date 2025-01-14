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

const size_t BUF_SIZE = 256;
const size_t ECHO_TASK_STACK_SIZE = 4096;
const size_t MAX_MESSAGE_LEN = 256;
const size_t SAMPLE_RATE = 10;
const size_t SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE;


// struct SensorConfig
// {
//     size_t id;
//     adc_channel_t sig_adc_channel;
//     gpio_num_t fbl_pin;
// };

// // sensor config for initializing. Should be done with a static array instead of using a dynamic
// SensorConfig sensor_config[] = {
//     {1, ADC_CHANNEL_5, GPIO_NUM_41},
//     {2, ADC_CHANNEL_4, GPIO_NUM_42},
//     {3, ADC_CHANNEL_3, GPIO_NUM_2},
//     {4, ADC_CHANNEL_6, GPIO_NUM_1},
//     {5, ADC_CHANNEL_7, GPIO_NUM_21},
//     {6, ADC_CHANNEL_9, GPIO_NUM_47},
//     {7, ADC_CHANNEL_8, GPIO_NUM_48},
//     {8, ADC_CHANNEL_2, GPIO_NUM_45},
// };

Config config = {
    .sensors {
        {.id=1, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_5, .fbl_pin=GPIO_NUM_41, .delayed=false, .x=60, .y=0},
        {.id=2, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_4, .fbl_pin=GPIO_NUM_42, .delayed=true, .x=120, .y=50},
        {.id=3, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_3, .fbl_pin=GPIO_NUM_2, .delayed=false, .x=180, .y=0},
        {.id=4, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_6, .fbl_pin=GPIO_NUM_1, .delayed=true, .x=240, .y=0},
        {.id=5, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_7, .fbl_pin=GPIO_NUM_21, .delayed=false, .x=300, .y=0},
        {.id=6, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_9, .fbl_pin=GPIO_NUM_47, .delayed=true, .x=360, .y=0},
        {.id=7, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_8, .fbl_pin=GPIO_NUM_48, .delayed=false, .x=420, .y=0},
        {.id=8, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_2, .fbl_pin=GPIO_NUM_45, .delayed=true, .x=480, .y=0},
    }
};

struct SamplerTaskParams
{
    QueueHandle_t queue;
    size_t num_sensors;
    Sensor** sensors;
};

struct SerialTaskParams
{
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
};


struct SensorResult
{
    size_t id;
    int64_t timestamp;
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

// Samples sensors in seperate task 
void sampler(void* parameters)
{
    auto params = static_cast<SamplerTaskParams*>(parameters);

    SamplerQueueItem queue_item;

    while (1) {
        memset(&queue_item, 0, sizeof(queue_item));
        int64_t current_time = esp_timer_get_time();

        queue_item.timestamp = current_time;

        // Sample the sensors
        for (int i = 0; i < params->num_sensors; i++)
        {
            queue_item.results[i].id = i + 1;
            queue_item.results[i].connected = params->sensors[i]->is_connected();
            queue_item.results[i].value = params->sensors[i]->sample();
            queue_item.results[i].timestamp = current_time;
        }

        // Verzenden naar hoofdqueue
        xQueueSend(params->queue, &queue_item, portMAX_DELAY);
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
    snprintf(buf, sizeof(buf), "$%lld:%s:%s#\n", timestamp, command, payload);
    
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

void send_sensor_measurements(const QueueHandle_t queue, const SensorResult* sensors)
{
    static char payload[MAX_MESSAGE_LEN];
    memset(payload, 0x00, sizeof(payload));

    int offset = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        const char* sep = (i==(NUM_SENSORS-1)) ? "" : ":"; 
        
        auto sensor = sensors[i];
        offset += snprintf(&payload[offset], sizeof(payload) - offset, "ID=%d:T=%lld:C=%d:V=%d%s", 
            sensor.id,
            sensor.timestamp,
            sensor.connected,
            sensor.value,
            sep);
    }

    // TODO: naam
    send_message(queue, "SMS", payload);
}

extern "C" void app_main(void)
{
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = BUF_SIZE,
        .rx_buffer_size = BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));

    printf("before ADC initialisation\n");
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,  // Gebruik ADC Unit 1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc1_handle));

    Sensor** sensors = new Sensor*[NUM_SENSORS];

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        sensors[i] = new Sensor(adc1_handle, config.sensors[i].sig_adc_channel, config.sensors[i].fbl_pin);
    }

    auto queue = xQueueCreate(5, sizeof(SamplerQueueItem));
    char rx_buf[MAX_MESSAGE_LEN];

    SamplerTaskParams sampler_params = {queue, NUM_SENSORS, sensors};
    xTaskCreate(sampler, "Sampler", 2048, &sampler_params, 5, nullptr);

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

    while(1)
    {
        // while loop to read mulitiple samples at once
        while (xQueueReceive(queue, &sampler_queue_item, pdMS_TO_TICKS(1))) {
            delay_buffer.push(sampler_queue_item);
            // printf("num delayed samples: %d, queue contains: %d\n", num_delayed_samples, items);

            // time between sensors * sample rate
            // groter dan, want items is hiervoor al gepushed, dus een item er af voor num_delayed_samples
            if (delay_buffer.size() > num_delayed_samples)
            {
                auto delayed_queue_item = delay_buffer.front();
                delay_buffer.pop();
                // printf("item timestamp: %lld - delayed item timestamp: %lld | DELTA: %lld\n", sampler_queue_item.results[0].timestamp, delayed_queue_item.results[0].timestamp, (sampler_queue_item.results[0].timestamp - delayed_queue_item.results[0].timestamp)/1000);

                const SensorResult* left_sensor = nullptr;
                const SensorResult* right_sensor = nullptr;

                for (size_t i = 0; i < NUM_SENSORS; i++)
                {
                    const SensorResult& result = config.sensors[i].delayed 
                        ? delayed_queue_item.results[i] 
                        : sampler_queue_item.results[i];

                    if (result.connected && result.value >= config.sensors[i].threshold)
                    {
                        if (!left_sensor || config.sensors[i].x < config.sensors[left_sensor->id - 1].x) {
                            left_sensor = &result;
                        }

                        if (!right_sensor || config.sensors[i].x > config.sensors[right_sensor->id - 1].x) {
                            right_sensor = &result;
                        }
                    }
                }

                // Bereken breedte als beide sensoren zijn gevonden
                if (left_sensor && right_sensor)
                {
                    int width = config.sensors[right_sensor->id - 1].x - config.sensors[left_sensor->id - 1].x + SENSOR_WIDTH;
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


   
            send_sensor_measurements(serial_tx_queue, sampler_queue_item.results);
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

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        delete sensors[i];
    }

    delete[] sensors;
}