#ifndef SERIAL_TASK_H
#define SERIAL_TASK_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct SerialTaskParams
{
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
};

void serial_task(void* parameters);

#endif