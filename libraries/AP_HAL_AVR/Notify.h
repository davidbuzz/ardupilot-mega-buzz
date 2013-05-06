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

    /// methods that the child instances should implement
    /// GPS_state - update gps lock state to 0:no gps, 1:no lock, 2:2d Lock, 3:3dLock
    virtual void _GPS_state(uint8_t state);
    /// arm_state - 0 = disarmed, 1 = disarmed+pre-arm-check failure, 2 = armed
    virtual void _arm_state(uint8_t state);
    /// _initialising - to indicate we should not move the vehicle
    virtual void _initialising(bool on_off);
    
private:

};
#endif	// __AP_HAL_AVR_NOTIFY_H__
