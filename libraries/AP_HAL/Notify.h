/*
 *  AP_HAL_AVR Notify Library. 
 *
 *  Copyright (c) 2013 David "Buzz" Bussenschutt.  All right reserved.
  *  Rev 1.0 - 1st March 2013
 *
  *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 */
#ifndef __AP_HAL_NOTIFY_H__
#define __AP_HAL_NOTIFY_H__

#include <stdint.h>

#include "AP_HAL_Namespace.h"

#define NOTIFY_MAX_OBJECTS      3

class AP_HAL::Notify
{
public:
    /// notify_type - bitmask of notification types
    struct notify_type {
        uint16_t gps            : 1;
        uint16_t arm            : 1;
        uint16_t initialising   : 1;
        	
				uint16_t recent_brownout   : 1;
				uint16_t low_battery   : 1;
				uint16_t accel_calibration_failure   : 1;
				uint16_t radio_calibration_failure   : 1;
				uint16_t compass_calibration_failure   : 1;
				uint16_t gyro_calibration_failure   : 1;
					
		    uint16_t fs_throttle   : 1;
		    uint16_t fs_battery   : 1;
		    uint16_t fs_gps   : 1;
		    uint16_t fs_gcs   : 1;
		    uint16_t fence_breach   : 1;
		    uint16_t switch_aux1   : 1;
		    uint16_t switch_aux2   : 1;
		    uint16_t reached_waypoint   : 1;
		    uint16_t flightmode   : 1;

    };

    /// Constructor - child instances should create their own
    Notify();

    ///
    /// external callers should call these methods to request notifcation updates to pilot
    ///

    /// GPS_state - update gps lock state to 0:no gps, 1:no lock, 2:2d Lock, 3:3dLock
    static void GPS_state(uint8_t state);

    /// arm_state - 0 = disarmed, 1 = disarmed+pre-arm-check failure, 2 = armed
    static void arm_state(uint8_t state);

    /// initialising - to indicate we should not move the vehicle
    static void initialising(bool on_off);

    // other things we can notify the user about, if the specific hardware supports it:   
    // none of these are "clearable" by passing in a bool, as they should only be cleared by a reboot. 
    static void recent_brownout();
    static void low_battery();
    static void accel_calibration_failure();
    static void radio_calibration_failure();
    static void compass_calibration_failure();
    static void gyro_calibration_failure();
    
    static void fs_throttle(bool uint8_t);     // 0 if throttle failsafe is cleared, 1 if activated
    static void fs_battery(bool uint8_t);      // 1 if battery voltage is low or consumed amps close to battery capacity, 0 if cleared
    static void fs_gps(bool uint8_t);          // 1 if we've lost gps lock and it is required for our current flightmode, 0 if cleared
    static void fs_gcs(bool uint8_t);          // 1 if we've lost contact with the gcs and it is required for our current flightmode or pilot input method, 0 if cleared
    static void fence_breach(bool uint8_t);    // fence type breached or 0 if cleared
    static void switch_aux1(uint8_t state);     // 0 if aux switch is off, 1 if in middle, 2 if high
    static void switch_aux2(uint8_t state);     // 0 if aux switch is off, 1 if in middle, 2 if high
    static void reached_waypoint();             // called right after we reach a waypoint
    static void flightmode(uint8_t mode);

    
protected:

    ///
    /// child instances should implement these methods
    ///

    /// GPS_state - update gps lock state to 0:no gps, 1:no lock, 2:2d Lock, 3:3dLock
    virtual void _GPS_state(uint8_t state) {};
    /// arm_state - 0 = disarmed, 1 = disarmed+pre-arm-check failure, 2 = armed
    virtual void _arm_state(uint8_t state) {};
    /// _initialising - to indicate we should not move the vehicle
    virtual void _initialising(bool on_off) {};

    // other things we can notify the user about, if the specific hardware supports it:   
    // none of these are "clearable" by passing in a bool, as they should only be cleared by a reboot. 
    virtual void _recent_brownout() {};
    virtual void _low_battery() {};
    virtual void _accel_calibration_failure() {};
    virtual void _radio_calibration_failure(){};
    virtual void _compass_calibration_failure() {};
    virtual void _gyro_calibration_failure() {};

    // todo improve names here  - what are we really trying to tell the user?  
    virtual void _fs_throttle(bool uint8_t){};     // 0 if throttle failsafe is cleared, 1 if activated
    virtual void _fs_battery(bool uint8_t){};      // 1 if battery voltage is low or consumed amps close to battery capacity, 0 if cleared
    virtual void _fs_gps(bool uint8_t){};          // 1 if we've lost gps lock and it is required for our current flightmode, 0 if cleared
    virtual void _fs_gcs(bool uint8_t){};          // 1 if we've lost contact with the gcs and it is required for our current flightmode or pilot input method, 0 if cleared
    virtual void _fence_breach(bool uint8_t){};    // fence type breached or 0 if cleared
    virtual void _switch_aux1(uint8_t state){};     // 0 if aux switch is off, 1 if in middle, 2 if high
    virtual void _switch_aux2(uint8_t state){};     // 0 if aux switch is off, 1 if in middle, 2 if high
    virtual void _reached_waypoint(){};             // called right after we reach a waypoint
    virtual void _flightmode(uint8_t mode){};


    
    /// id of this instance - used as an index into the children and interests tables
    uint8_t _id;

    ///
    /// methods that should not be overwritten
    ///

    /// set_interests - register notification types that this object supports
    void set_interests(notify_type interests) { _interests[_id] = interests; }

    /// table of all instantiated notify objects
    static Notify* _children[NOTIFY_MAX_OBJECTS];
    static notify_type _interests[NOTIFY_MAX_OBJECTS];
    static uint8_t _num_children;
};

#endif // __AP_HAL_NOTIFY_H__

