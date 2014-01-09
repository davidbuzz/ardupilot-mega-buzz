/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_HAL_Quanton_ANALOGIN_H__
#define __AP_HAL_Quanton_ANALOGIN_H__

#include <AP_HAL_Quanton.h>
#include <pthread.h>
#include <uORB/uORB.h>

#define Quanton_ANALOG_MAX_CHANNELS 16


#ifdef CONFIG_ARCH_BOARD_Quanton
// these are virtual pins that read from the ORB
#define QUANTON_ANALOG_ORB_BATTERY_VOLTAGE_PIN     100
#define QUANTON_ANALOG_ORB_BATTERY_CURRENT_PIN     101
#endif

class Quanton::QuantonAnalogSource : public AP_HAL::AnalogSource {
public:
    friend class Quanton::QuantonAnalogIn;
    QuantonAnalogSource(int16_t pin, float initial_value);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();

    // stop pins not implemented on Quanton yet
    void set_stop_pin(uint8_t p) {}
    void set_settle_time(uint16_t settle_time_ms) {}

private:
    // what pin it is attached to
    int16_t _pin;

    // what value it has
    float _value;
    float _value_ratiometric;
    float _latest_value;
    uint8_t _sum_count;
    float _sum_value;
    float _sum_ratiometric;
    void _add_value(float v, uint16_t vcc5V_mV);
    float _pin_scaler();
};

class Quanton::QuantonAnalogIn : public AP_HAL::AnalogIn {
public:
    QuantonAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t pin);
    void _timer_tick(void);

private:
    int _adc_fd;
    int _battery_handle;
    int _servorail_handle;
    uint64_t _battery_timestamp;
    uint64_t _servorail_timestamp;
    Quanton::QuantonAnalogSource* _channels[Quanton_ANALOG_MAX_CHANNELS];
    uint32_t _last_run;
};
#endif // __AP_HAL_Quanton_ANALOGIN_H__
