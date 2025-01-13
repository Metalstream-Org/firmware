#pragma once

#include <stdint.h>
#include <esp_adc/adc_oneshot.h>
#include "driver/gpio.h"

const size_t MAX_NUM_SENSORS = 8;

typedef struct {
    size_t id;
    int threshold;
    adc_channel_t sig_adc_channel;
    gpio_num_t fbl_pin;
    int x;
    int y;
} SensorConfig;

typedef struct {
    SensorConfig sensors[8];
} Config;

Config config = {
    .sensors {
        {.id=1, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_5, .fbl_pin=GPIO_NUM_41, .x=1, .y=0},
        {.id=2, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_4, .fbl_pin=GPIO_NUM_42, .x=1, .y=0},
        {.id=3, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_3, .fbl_pin=GPIO_NUM_2, .x=1, .y=0},
        {.id=4, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_6, .fbl_pin=GPIO_NUM_1, .x=1, .y=0},
        {.id=5, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_7, .fbl_pin=GPIO_NUM_21, .x=1, .y=0},
        {.id=6, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_9, .fbl_pin=GPIO_NUM_47, .x=1, .y=0},
        {.id=7, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_8, .fbl_pin=GPIO_NUM_48, .x=1, .y=0},
        {.id=8, .threshold=1023, .sig_adc_channel=ADC_CHANNEL_2, .fbl_pin=GPIO_NUM_45, .x=1, .y=0},
    }
};
