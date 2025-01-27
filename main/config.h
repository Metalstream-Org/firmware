#pragma once

#include <stdint.h>
#include <esp_adc/adc_oneshot.h>
#include "driver/gpio.h"

const size_t NUM_SENSORS = 8;
// mm
const size_t SENSOR_WIDTH = 100;
// mm
const size_t SENSOR_LONGITUDINAL_SAPCING = 100;

// De grootte van de USB FIFO buffer
const size_t USB_FIFO_BUF_SIZE = 128;
// De maximale lengte van een bericht
const size_t MAX_MESSAGE_LEN = 128;
// Aantal keer dat we per seconde willen samplen
const size_t SAMPLE_RATE = 20;
// Sample interval
const size_t SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE;

const float INITIAL_CONVEYER_BELT_SPEED = 9.1;