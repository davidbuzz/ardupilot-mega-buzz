/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <Notify.h>

extern const AP_HAL::HAL& hal;

uint8_t AP_HAL::Notify::_num_children = 0;
AP_HAL::Notify* AP_HAL::Notify::_children[NOTIFY_MAX_OBJECTS];
AP_HAL::Notify::notify_type AP_HAL::Notify::_interests[NOTIFY_MAX_OBJECTS];

/// Constructor
AP_HAL::Notify::Notify()
{
    // get notifier id and set-up children table
    _id = _num_children++;
    _children[_id] = this;
}

///
/// external callers should call these methods to request notifcation updates to pilot
///

/// GPS_state - update gps lock state to 0:no gps, 1:no lock, 2:2d Lock, 3:3dLock
void AP_HAL::Notify::GPS_state(uint8_t state)
{
    // call children who support this notification
    for(uint8_t i=0; i<_num_children; i++) {
        if( _interests[i].gps ) {
            _children[i]->_GPS_state(state);
        }
    }
}

/// arm_state - 0 = disarmed, 1 = disarmed+pre-arm-check failure, 2 = armed
void AP_HAL::Notify::arm_state(uint8_t state) {
    // call children who support this notification
    for(uint8_t i=0; i<_num_children; i++) {
        if( _interests[i].gps ) {
            _children[i]->_arm_state(state);
        }
    }
}

/// initialising - to indicate we should not move the vehicle
void AP_HAL::Notify::initialising(bool on_off) {
    // call children who support this notification
    for(uint8_t i=0; i<_num_children; i++) {
        if( _interests[i].initialising ) {
            _children[i]->_initialising(on_off);
        }
    }
}

void AP_HAL::Notify::recent_brownout() {
    // call children who support this notification
    for(uint8_t i=0; i<_num_children; i++) {
        if( _interests[i].recent_brownout ) {
            _children[i]->_recent_brownout();
        }
    }
}
void AP_HAL::Notify::low_battery() {
    // call children who support this notification
    for(uint8_t i=0; i<_num_children; i++) {
        if( _interests[i].low_battery ) {
            _children[i]->_low_battery();
        }
    }
}
void AP_HAL::Notify::accel_calibration_failure() {
    // call children who support this notification
    for(uint8_t i=0; i<_num_children; i++) {
        if( _interests[i].accel_calibration_failure ) {
            _children[i]->_accel_calibration_failure();
        }
    }
}
void AP_HAL::Notify::radio_calibration_failure() {
    // call children who support this notification
    for(uint8_t i=0; i<_num_children; i++) {
        if( _interests[i].radio_calibration_failure ) {
            _children[i]->_radio_calibration_failure();
        }
    }
}
void AP_HAL::Notify::compass_calibration_failure() {
    // call children who support this notification
    for(uint8_t i=0; i<_num_children; i++) {
        if( _interests[i].compass_calibration_failure ) {
            _children[i]->_compass_calibration_failure();
        }
    }
}
void AP_HAL::Notify::gyro_calibration_failure() {
    // call children who support this notification
    for(uint8_t i=0; i<_num_children; i++) {
        if( _interests[i].gyro_calibration_failure ) {
            _children[i]->_gyro_calibration_failure();
        }
    }
}