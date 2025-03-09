#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>

#include "sensor.h"

Sensor::Sensor(size_t id, int threshold, adc_oneshot_unit_handle_t adc_handle, adc_channel_t sig_adc_channel, gpio_num_t fbl_pin, bool delayed, int x) : m_id{id}, m_threshold{threshold}, m_adc_handle{adc_handle}, m_sig_adc_channel{sig_adc_channel}, m_fbl_pin{fbl_pin}, m_delayed{delayed}, m_x{x}
{
    // Configureer de feedbackloop pin als input en enable de pullup weerstand.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << fbl_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Configureer het ADC channel op basis van de signaal pin.
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(m_adc_handle, m_sig_adc_channel, &config));
}

// Haalt de connected status op van de sensor.
bool Sensor::is_connected() const
{
    return gpio_get_level(m_fbl_pin) == 0;
}

// Helper functie om te checken of de waarde boven de sensor threshold ligt.
bool Sensor::is_value_above_threshold(uint16_t value)
{
    return value > m_threshold;   
}

bool Sensor::is_delayed() const
{
    return m_delayed;
}

int Sensor::get_x() const
{
    return m_x;
}

uint16_t Sensor::sample()
{
    int value;
    adc_oneshot_read(m_adc_handle, m_sig_adc_channel, &value);
    return value;
}