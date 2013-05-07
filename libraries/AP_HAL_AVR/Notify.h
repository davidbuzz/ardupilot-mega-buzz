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
    
    
    
private:

};
#endif	// __AP_HAL_AVR_NOTIFY_H__
