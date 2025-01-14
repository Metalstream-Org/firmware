#pragma once

#include <stdint.h>
#include <esp_adc/adc_oneshot.h>
#include "driver/gpio.h"

const size_t NUM_SENSORS = 8;
const size_t SENSOR_WIDTH = 100;

typedef struct {
    size_t id;
    int threshold;
    adc_channel_t sig_adc_channel;
    gpio_num_t fbl_pin;
    bool delayed;
    int x;
    int y;
} SensorConfig;

typedef struct {
    SensorConfig sensors[NUM_SENSORS];
} Config;
