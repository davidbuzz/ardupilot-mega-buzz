/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include "Notify.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

/// Constructor
AVRNotify::AVRNotify()
{
    // indicate that we support the following notification types
    _interests[_id].gps = true;
    _interests[_id].arm = true;
    _interests[_id].initialising = true;
}

/// _GPS_state - update gps lock state to 0:no gps, 1:no lock, 2:2d Lock, 3:3dLock
void AVRNotify::_GPS_state(uint8_t state)
{
    // do something
}

/// _arm_state - 0 = disarmed, 1 = disarmed+pre-arm-check failure, 2 = armed
void AVRNotify::_arm_state(uint8_t state) {
    // do something
}

/// _initialising - to indicate we should not move the vehicle
void AVRNotify::_initialising(bool on_off) {
    // do something
}