#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <esp_adc/adc_oneshot.h>
#include <hal/adc_types.h>
#include <driver/gpio.h>

class Sensor {
    private:
        size_t m_id;                                    // Het sensor ID.
        int m_threshold;                                // De threshold waarde voor de sensor.
        adc_oneshot_unit_handle_t m_adc_handle;         // De ADC handle.
        adc_channel_t m_sig_adc_channel;                // Het ADC channel voor het signaal.
        gpio_num_t m_fbl_pin;                           // De GPIO pin voor de feedbackloop.
        bool m_delayed;                                 // Geeft aan of de sensor delayed is of niet (onderste en bovenste rij).
        int m_x;                                        // De x-positie van de sensor.
    public:
        Sensor(size_t id, int threshold, adc_oneshot_unit_handle_t adc_handle, adc_channel_t sig_adc_channel, gpio_num_t fbl_pin, bool delayed, int x);
        bool is_connected() const;
        bool is_value_above_threshold(uint16_t value);
        bool is_delayed() const;
        int get_x() const;
        uint16_t sample();
};

#endif