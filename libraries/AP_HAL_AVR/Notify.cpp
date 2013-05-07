/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include "Notify.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

/// Constructor
AVRNotify::AVRNotify()
{
    // indicate that we support the following common notification types
    _interests[_id].gps = true;
    _interests[_id].arm = true;
    _interests[_id].initialising = true;
    // less common ones:  
		_interests[_id].recent_brownout = true;
		_interests[_id].low_battery = true;
		_interests[_id].accel_calibration_failure = true;
		_interests[_id].radio_calibration_failure = true;
		_interests[_id].compass_calibration_failure = true;
		_interests[_id].gyro_calibration_failure = true;

}

/// _GPS_state - update gps lock state to 0:no gps, 1:no lock, 2:2d Lock, 3:3dLock
void AVRNotify::_GPS_state(uint8_t state){
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

// other things we can notify the user about, if the specific hardware supports it:   
// none of these are "clearable" by passing in a bool, as they should only be cleared by a reboot. 
void AVRNotify::_recent_brownout() {
    // do something
}
void AVRNotify::_low_battery() {
    // do something
}
void AVRNotify::_accel_calibration_failure() {
    // do something
}
void AVRNotify::_radio_calibration_failure() {
    // do something
}
void AVRNotify::_compass_calibration_failure() {
    // do something
}
void AVRNotify::_gyro_calibration_failure() {
    // do something
}

void AVRNotify::_fs_throttle(bool uint8_t) {
    // do something
}
void AVRNotify::_fs_battery(bool uint8_t) {
    // do something
}
void AVRNotify::_fs_gps(bool uint8_t) {
    // do something
}
void AVRNotify::_fs_gcs(bool uint8_t) {
    // do something
}
void AVRNotify::_fence_breach(bool uint8_t) {
    // do something
}
void AVRNotify::_switch_aux1(uint8_t state) {
    // do something
}
void AVRNotify::_switch_aux2(uint8_t state) {
    // do something
}
void AVRNotify::_reached_waypoint() {
    // do something
}
void AVRNotify::_flightmode(uint8_t mode) {
    // do something
}
