#ifndef SAMPLER_TASK_H
#define SAMPLER_TASK_H

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "config.h"
#include "sensor.h"

struct SensorResult
{
    size_t id;
    bool connected;
    uint16_t value;
};

struct SamplerQueueItem
{
    int64_t timestamp;
    SensorResult results[NUM_SENSORS];
};

struct SamplerTaskParams
{
    QueueHandle_t queue;
    size_t num_sensors;
    Sensor* sensors;
};

void sampler_task(void* parameters);

#endif