#pragma once

#include <stdint.h>
#include <esp_adc/adc_oneshot.h>
#include "driver/gpio.h"

const size_t NUM_SENSORS = 8;

typedef struct {
    size_t id;
    int threshold;
    adc_channel_t sig_adc_channel;
    gpio_num_t fbl_pin;
    int x;
    int y;
} SensorConfig;

typedef struct {
    SensorConfig sensors[NUM_SENSORS];
} Config;
