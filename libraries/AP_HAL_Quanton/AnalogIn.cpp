/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUANTON

#include "AnalogIn.h"
#include <drivers/drv_adc.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/config.h>
#include <arch/board/board.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/servorail_status.h>

#define ANLOGIN_DEBUGGING 0

// base voltage scaling for 12 bit 3.3V ADC
#define Quanton_VOLTAGE_SCALING (3.3f/4096.0f)

#if ANLOGIN_DEBUGGING
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

/*
  scaling table between ADC count and actual input voltage, to account
  for voltage dividers on the board. 
 */
static const struct {
    uint8_t pin;
    float scaling;
} pin_scaling[] = {

    // Quanton has 2 FMU analog input pins?   TODO BUZZ CHANGE /CHECK PIN MAP! 
    { 10, (5.7*3.3)/4096 }, // FMU battery on multi-connector pin 5,
                            // 5.7:1 scaling
    { 11,  6.6f/4096  }, // analog airspeed input, 2:1 scaling
    { 12,  3.3f/4096  }, // analog2, on SPI port pin 3
    { 13, 16.8f/4096  }, // analog3, on SPI port pin 4

};

using namespace Quanton;

QuantonAnalogSource::QuantonAnalogSource(int16_t pin, float initial_value) :
	_pin(pin),
    _value(initial_value),
    _value_ratiometric(initial_value),
    _latest_value(initial_value),
    _sum_count(0),
    _sum_value(0),
    _sum_ratiometric(0)
{
#ifdef Quanton_ANALOG_VCC_5V_PIN
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        _pin = Quanton_ANALOG_VCC_5V_PIN;
    }
#endif
}

float QuantonAnalogSource::read_average() 
{
    if (_sum_count == 0) {
        return _value;
    }
    hal.scheduler->suspend_timer_procs();
    _value = _sum_value / _sum_count;
    _value_ratiometric = _sum_ratiometric / _sum_count;
    _sum_value = 0;
    _sum_ratiometric = 0;
    _sum_count = 0;
    hal.scheduler->resume_timer_procs();
    return _value;
}

float QuantonAnalogSource::read_latest() 
{
    return _latest_value;
}

/*
  return scaling from ADC count to Volts
 */
float QuantonAnalogSource::_pin_scaler(void)
{
    float scaling = Quanton_VOLTAGE_SCALING;
    uint8_t num_scalings = sizeof(pin_scaling)/sizeof(pin_scaling[0]);
    for (uint8_t i=0; i<num_scalings; i++) {
        if (pin_scaling[i].pin == _pin) {
            scaling = pin_scaling[i].scaling;
            break;
        }
    }
    return scaling;
}

/*
  return voltage in Volts
 */
float QuantonAnalogSource::voltage_average()
{
    return _pin_scaler() * read_average();
}

/*
  return voltage in Volts, assuming a ratiometric sensor powered by
  the 5V rail
 */
float QuantonAnalogSource::voltage_average_ratiometric()
{
    voltage_average();
    return _pin_scaler() * _value_ratiometric;
}

/*
  return voltage in Volts
 */
float QuantonAnalogSource::voltage_latest()
{
    return _pin_scaler() * read_latest();
}

void QuantonAnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }
    hal.scheduler->suspend_timer_procs();
    _pin = pin;
    _sum_value = 0;
    _sum_ratiometric = 0;
    _sum_count = 0;
    _latest_value = 0;
    _value = 0;
    _value_ratiometric = 0;
    hal.scheduler->resume_timer_procs();
}

/*
  apply a reading in ADC counts
 */
void QuantonAnalogSource::_add_value(float v, uint16_t vcc5V_mV)
{
    _latest_value = v;
    _sum_value += v;
    if (vcc5V_mV == 0) {
        _sum_ratiometric += v;
    } else {
        // this compensates for changes in the 5V rail relative to the
        // 3.3V reference used by the ADC.
        _sum_ratiometric += v * 5000 / vcc5V_mV;
    }
    _sum_count++;
    if (_sum_count == 254) {
        _sum_value /= 2;
        _sum_ratiometric /= 2;
        _sum_count /= 2;
    }
}


QuantonAnalogIn::QuantonAnalogIn()
{}

void QuantonAnalogIn::init(void* machtnichts)
{
	_adc_fd = open(ADC_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
    if (_adc_fd == -1) {
        hal.scheduler->panic("Unable to open " ADC_DEVICE_PATH);
	}
    _battery_handle   = orb_subscribe(ORB_ID(battery_status));
    _servorail_handle = orb_subscribe(ORB_ID(servorail_status));
}

/*
  called at 1kHz
 */
void QuantonAnalogIn::_timer_tick(void)
{
    // read adc at 100Hz
    uint32_t now = hal.scheduler->micros();
    uint32_t delta_t = now - _last_run;
    if (delta_t < 10000) {
        return;
    }
    _last_run = now;

    struct adc_msg_s buf_adc[Quanton_ANALOG_MAX_CHANNELS];

    /* read all channels available */
    int ret = read(_adc_fd, &buf_adc, sizeof(buf_adc));
    if (ret > 0) {
        uint16_t vcc5V_mV = 0;
        // match the incoming channels to the currently active pins
        for (uint8_t i=0; i<ret/sizeof(buf_adc[0]); i++) {

        }
        for (uint8_t i=0; i<ret/sizeof(buf_adc[0]); i++) {
            Debug("chan %u value=%u\n",
                  (unsigned)buf_adc[i].am_channel,
                  (unsigned)buf_adc[i].am_data);
            for (uint8_t j=0; j<Quanton_ANALOG_MAX_CHANNELS; j++) {
                Quanton::QuantonAnalogSource *c = _channels[j];
                if (c != NULL && buf_adc[i].am_channel == c->_pin) {
                    c->_add_value(buf_adc[i].am_data, vcc5V_mV);
                }
            }
        }
    }

    // check for new battery data on Quanton
    if (_battery_handle != -1) {
        struct battery_status_s battery;
        if (orb_copy(ORB_ID(battery_status), _battery_handle, &battery) == OK &&
            battery.timestamp != _battery_timestamp) {
            _battery_timestamp = battery.timestamp;
            for (uint8_t j=0; j<Quanton_ANALOG_MAX_CHANNELS; j++) {
                Quanton::QuantonAnalogSource *c = _channels[j];
                if (c == NULL) continue;
                if (c->_pin == QUANTON_ANALOG_ORB_BATTERY_VOLTAGE_PIN) {
                    c->_add_value(battery.voltage_v / Quanton_VOLTAGE_SCALING, 0);
                }
                if (c->_pin == QUANTON_ANALOG_ORB_BATTERY_CURRENT_PIN) {
                    // scale it back to voltage, knowing that the
                    // Quantonio code scales by 90.0/5.0
                    c->_add_value(battery.current_a * (5.0f/90.0f) / Quanton_VOLTAGE_SCALING, 0);
                }
            }
        }
    }

}

AP_HAL::AnalogSource* QuantonAnalogIn::channel(int16_t pin) 
{
    for (uint8_t j=0; j<Quanton_ANALOG_MAX_CHANNELS; j++) {
        if (_channels[j] == NULL) {
            _channels[j] = new QuantonAnalogSource(pin, 0.0);
            return _channels[j];
        }
    }
    hal.console->println("Out of analog channels");
    return NULL;
}

#endif // CONFIG_HAL_BOARD
