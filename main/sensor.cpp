#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>

#include "sensor.h"

Sensor::Sensor(size_t id, int threshold, adc_oneshot_unit_handle_t adc_handle, adc_channel_t sig_adc_channel, gpio_num_t fbl_pin, bool delayed, int x, int y) : m_id{id}, m_threshold{threshold}, m_adc_handle{adc_handle}, m_sig_adc_channel{sig_adc_channel}, m_fbl_pin{fbl_pin}, m_delayed{delayed}, m_x{x}, m_y{y}
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << fbl_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // todo for later maby: static constructor for config width etc.

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(m_adc_handle, m_sig_adc_channel, &config));
}

bool Sensor::is_connected() const
{
    return gpio_get_level(m_fbl_pin) == 0;
}

int Sensor::get_threshold() const
{
    return m_threshold;   
}

bool Sensor::is_delayed() const
{
    return m_delayed;
}

int Sensor::get_x() const
{
    return m_x;
}

int Sensor::get_y() const
{
    return m_y;
}

uint16_t Sensor::sample()
{
    int value;
    adc_oneshot_read(m_adc_handle, m_sig_adc_channel, &value);
    return value;
}