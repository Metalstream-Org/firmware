#pragma once
#include <stdint.h>
#include <esp_adc/adc_oneshot.h>
#include <hal/adc_types.h>
#include <driver/gpio.h>

class Sensor {
    private:
        // todo: better name
        const int NUM_SAMPLES = 10;
        adc_oneshot_unit_handle_t m_adc_handle;
        adc_channel_t m_sig_adc_channel;
        gpio_num_t m_fbl_pin;
    public:
        Sensor(adc_oneshot_unit_handle_t adc_handle, adc_channel_t sig_adc_channel, gpio_num_t fbl_pin);
        bool is_connected();
        uint16_t sample();
};