#include <stdio.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <esp_adc/adc_oneshot.h>
#include "string.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sensor.h"

// Definities voor de GPIO pins
#define GPIO_INPUT_PIN 41  // Digital input
#define ADC_INPUT_CHANNEL ADC1_CHANNEL_5
#define ADC_WIDTH ADC_WIDTH_BIT_12       // 12-bit resolutie
#define ADC_ATTEN ADC_ATTEN_DB_12        // Attenuatie voor een breder meetbereik

struct SensorConfig
{
    size_t id;
    adc_channel_t sig_adc_channel;
    gpio_num_t fbl_pin;
};

// sensor config for initializing. Should be done with a static array instead of using a dynamic
SensorConfig config[] = {
    {1, ADC_CHANNEL_5, GPIO_NUM_41},
    {2, ADC_CHANNEL_4, GPIO_NUM_42},
    {3, ADC_CHANNEL_3, GPIO_NUM_2},
    {4, ADC_CHANNEL_6, GPIO_NUM_1},
    {5, ADC_CHANNEL_7, GPIO_NUM_21},
    {6, ADC_CHANNEL_9, GPIO_NUM_47},
    {7, ADC_CHANNEL_8, GPIO_NUM_48},
    {8, ADC_CHANNEL_2, GPIO_NUM_45},
};

const size_t NUM_SENSORS = sizeof(config) / sizeof(config[0]);

struct SamplerTaskParams
{
    QueueHandle_t queue;
    size_t num_sensors;
    Sensor** sensors;
};

struct SensorResult
{
    int64_t timestamp;
    bool connected;
    int value;
};

const size_t MAX_NUM_SENSORS = 8;

struct SamplerQueueItem
{
    SensorResult results[MAX_NUM_SENSORS];
};

void sampler(void* parameters)
{
    auto params = static_cast<SamplerTaskParams*>(parameters);

    SamplerQueueItem queue_item;

    while (1) {
        printf("hello loop\n");
        memset(&queue_item, 0, sizeof(queue_item));

        for (int i = 0; i < params->num_sensors; i++)
        {
            queue_item.results[i].connected = params->sensors[i]->is_connected();
            queue_item.results[i].value = params->sensors[i]->sample();
            queue_item.results[i].timestamp = esp_timer_get_time();

            // printf("S0%d - connected: %d, sampled: %d\n", i+1, params->sensors[i]->is_connected(), params->sensors[i]->sample());
        }

        xQueueSend(params->queue, &queue_item, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(500));  // Wacht 500 ms
    }
}

extern "C" void app_main(void)
{
    const tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    tusb_cdc_acm_init(&acm_cfg);



    auto queue = xQueueCreate(3, sizeof(SamplerQueueItem));
    printf("before ADC initialisation\n");
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,  // Gebruik ADC Unit 1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc1_handle));

    Sensor** sensors = new Sensor*[NUM_SENSORS];

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        sensors[i] = new Sensor(adc1_handle, config[i].sig_adc_channel, config[i].fbl_pin);
    }

    SamplerTaskParams params = {queue, NUM_SENSORS, sensors};
    xTaskCreate(sampler, "Sampler", 2048, &params, 5, nullptr);

    while(1)
    {
        SamplerQueueItem sampler_queue_item;

        // $:timestamp:payload:checksum:#

        if (xQueueReceive(queue, &sampler_queue_item, portMAX_DELAY)) {
            for (int i = 0; i < NUM_SENSORS; i++)
            {
                printf("S0%d - timestamp: %lld, connected: %d, sampled: %d\n", i+1, sampler_queue_item.results[i].timestamp, sampler_queue_item.results[i].connected, sampler_queue_item.results[i].value);
            }

            // Verwerking of opslaan
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }


    // printf("before loop\n");
    // Eindeloze loop om zowel de digitale input als de ADC-waarde uit te lezen
    // while (1) {
    //     printf("hello loop\n");
    //     // Lees de digitale input van GPIO 41
    //     // int pin_state = gpio_get_level((gpio_num_t)GPIO_INPUT_PIN);
    //     // printf("GPIO 41 state: %d\n", pin_state);

    //     // // Lees de ADC-waarde van GPIO 6
    //     // int adc_value = adc1_get_raw(ADC_INPUT_CHANNEL);
    //     // printf("ADC waarde van GPIO 6: %d\n", adc_value);

    //     for (int i = 0; i < NUM_SENSORS; i++)
    //     {
    //         printf("S0%d - connected: %d, sampled: %d\n", i+1, sensors[i]->is_connected(), sensors[i]->sample());
    //     }

    //     vTaskDelay(pdMS_TO_TICKS(500));  // Wacht 500 ms
    // }

    // Clean up sensors when deinitializing
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        delete sensors[i];
    }

    delete[] sensors;
}