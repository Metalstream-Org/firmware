#include <esp_timer.h>
#include <string.h>

#include "sampler_task.h"

// Het nemen van samples wordt in een losse task gedaan wegens effecientie.
// Een FreeRTOS task is veel beter in het schedulen, waardoor we vaker en nauwkeuriger kunnen samplen.
// Het samplen is best tijd kritisch, omdat op basis van de SAMPLE_RATE de delay buffer grootte wordt bepaald.
void sampler_task(void* parameters)
{
    auto params = static_cast<SamplerTaskParams*>(parameters);

    SamplerQueueItem queue_item;

    while (true) {
        // Hier wordt gebruik van een memset, omdat dit een stuk effecienter is als het telkens opnieuw initaliseren van queue_item.
        memset(&queue_item, 0x00, sizeof(queue_item));
        queue_item.timestamp = esp_timer_get_time();

        // Elke sensor sampled en slaat de huidige toestand op in queue_item.
        for (int i = 0; i < params->num_sensors; i++)
        {
            queue_item.results[i].id = i + 1;
            queue_item.results[i].connected = params->sensors[i].is_connected();
            queue_item.results[i].value = params->sensors[i].sample();
        }

        // Verzend het sample naar de queue.
        xQueueSend(params->queue, &queue_item, portMAX_DELAY);
        // Hier wordt gebruik gemaakt van een delay om te zorgen dat de task sampled op de juiste frequentie (SAMPLE_RATE).
        // Waar een normale delay invloed heeft op de rest van de main loop, is dat hier niet het geval. FreeRTOS scheduled op basis hiervan.
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}