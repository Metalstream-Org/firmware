#pragma once
#include <stdint.h>
#include <esp_adc/adc_oneshot.h>
#include <hal/adc_types.h>
#include <driver/gpio.h>

class Sensor {
    private:
        // todo: better name
        const int NUM_SAMPLES = 10;
        size_t m_id;
        int m_threshold;
        adc_oneshot_unit_handle_t m_adc_handle;
        adc_channel_t m_sig_adc_channel;
        gpio_num_t m_fbl_pin;
        bool m_delayed;
        int m_x;
        int m_y;
    public:
        Sensor(size_t id, int threshold, adc_oneshot_unit_handle_t adc_handle, adc_channel_t sig_adc_channel, gpio_num_t fbl_pin, bool delayed, int x, int y);
        bool is_connected() const;
        int get_threshold() const;
        bool is_delayed() const;
        int get_x() const;
        int get_y() const;
        uint16_t sample();
};