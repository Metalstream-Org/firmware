#pragma once

#include <stdint.h>
#include <esp_adc/adc_oneshot.h>
#include "driver/gpio.h"

// Het aantal sensor IO poorten op de Metalstream Hub
const size_t NUM_SENSORS = 8;
// De breedte van een sensor in mm
const size_t SENSOR_WIDTH = 100;
// De longitudinale afstand tussen de sensoren in mm
const size_t SENSOR_LONGITUDINAL_SAPCING = 100;

// De grootte van de USB FIFO buffer
const size_t USB_FIFO_BUF_SIZE = 128;
// De maximale lengte van een bericht dat wordt over gestuurd over het protocol
const size_t MAX_MESSAGE_LEN = 128;
// Aantal samples per seconde
const size_t SAMPLE_RATE = 20;
// De interval tussen samples in ms
const size_t SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE;
// De maximale tijd die een kalibratie mag duren in ms
const size_t MAX_CALIBRATION_TIME_MS = 20000;

// De initiële snelheid van de lopende band in cm/s
const float INITIAL_CONVEYER_BELT_SPEED = 9.5f;