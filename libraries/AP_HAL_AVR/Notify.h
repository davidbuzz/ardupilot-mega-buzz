/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_HAL_AVR_NOTIFY_H__
#define __AP_HAL_AVR_NOTIFY_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::AVRNotify : public AP_HAL::Notify
{
public:
    /// Constructor
    AVRNotify();

protected:
	
	  // we wnt ot tell the user about stuff related to GPS, arming state, initialisation, and critical errors

    /// methods that the child instances should implement
    /// _GPS_state - update gps lock state to 0:no gps, 1:no lock, 2:2d Lock, 3:3dLock
    virtual void _GPS_state(uint8_t state);
    /// _arm_state - 0 = disarmed, 1 = disarmed+pre-arm-check failure, 2 = armed
    virtual void _arm_state(uint8_t state);
    /// _initialising - to indicate we should not move the vehicle
    virtual void _initialising(bool on_off);
    // other things we can notify the user about, if the specific hardware supports it:   
    // none of these are "clearable" by passing in a bool, as they should only be cleared by a reboot. 
    virtual void _recent_brownout();
    virtual void _low_battery();
    virtual void _accel_calibration_failure();
    virtual void _radio_calibration_failure();
    virtual void _compass_calibration_failure();
    virtual void _gyro_calibration_failure();
		// what other subsystems can we detect failure on ( and tell user about with POST code/s) 
		// , or need to blink led for any other reason, or play/beep tones, or send a msg, or...? 
    
    virtual void _fs_throttle(bool uint8_t);     // 0 if throttle failsafe is cleared, 1 if activated
    virtual void _fs_battery(bool uint8_t);      // 1 if battery voltage is low or consumed amps close to battery capacity, 0 if cleared
    virtual void _fs_gps(bool uint8_t);          // 1 if we've lost gps lock and it is required for our current flightmode, 0 if cleared
    virtual void _fs_gcs(bool uint8_t);          // 1 if we've lost contact with the gcs and it is required for our current flightmode or pilot input method, 0 if cleared
    virtual void _fence_breach(bool uint8_t);    // fence type breached or 0 if cleared
    virtual void _switch_aux1(uint8_t state);     // 0 if aux switch is off, 1 if in middle, 2 if high
    virtual void _switch_aux2(uint8_t state);     // 0 if aux switch is off, 1 if in middle, 2 if high
    virtual void _reached_waypoint();             // called right after we reach a waypoint
    virtual void _flightmode(uint8_t mode);

    
    
private:

};
#endif	// __AP_HAL_AVR_NOTIFY_H__
