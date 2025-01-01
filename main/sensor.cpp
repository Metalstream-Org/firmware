#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>

#include "sensor.h"

Sensor::Sensor(adc_oneshot_unit_handle_t adc_handle, adc_channel_t sig_adc_channel, gpio_num_t fbl_pin) : m_adc_handle{adc_handle}, m_sig_adc_channel{sig_adc_channel}, m_fbl_pin{fbl_pin}
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

bool Sensor::is_connected()
{
    return gpio_get_level(m_fbl_pin) == 0;
}

uint16_t Sensor::sample()
{
    int value;
    adc_oneshot_read(m_adc_handle, m_sig_adc_channel, &value);
    return value;
}