#pragma once

#include <stdint.h>
#include <esp_adc/adc_oneshot.h>
#include "driver/gpio.h"

const size_t NUM_SENSORS = 8;
const size_t SENSOR_WIDTH = 100;
// mm
const size_t SENSOR_LONGITUDINAL_SAPCING = 100;