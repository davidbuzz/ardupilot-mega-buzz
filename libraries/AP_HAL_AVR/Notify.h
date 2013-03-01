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
#ifndef __AP_HAL_AVR_NOTIFY_H__
#define __AP_HAL_AVR_NOTIFY_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

// This enum describes the possible "states" the RGB BlinkM LED might be put into.   it's a WIP. 
enum NotifyType { 
                  POWERON, 
                  RGB_FAST,  // siren-like-attention-getter ( very fast, 1.5sec => 15 transitions) 
                  RGB_SLOW,  // slow and docile at ~1/6th Hz ( 2 secs => 1 transition ) 
                  
                  RED_SOLID_MOTORS_ARMED, //A copter 
                  RED_FLASH_MOTORS_DISARMED, //A  copter
                  RED_GREEN_FAST_FLASH_CALIBRATE_GYRO,  //A = green, 
                  YELLOW_FLASH_CALIBRATING,  // plane and copter, off rest of the time on copter.
                  YELLOW_SOLID_READY_EXCEPT_FOR_GPS_LOCK,   // B
                  BLUE_FLASH_NO_GPS_LOCK,    //C  
                  BLUE_SOLID_GPS_LOCK_OK,   //C  
                  

                  CALIBRATION_ROUTINE, // All LEDS blinking in sequence means you are in the one-time ESC calibration routine
                  
                  PURPLE // for future use. 
                  };

class AP_HAL_AVR::AVRNotify : public AP_HAL::Notify {
public:
    AVRNotify() {}


    void set(uint8_t val); 
    void setrgb(uint8_t r, uint8_t g, uint8_t b); 
        
    void notify(uint8_t n); 

     void BlinkM_stopScript(uint8_t addr);
     void BlinkM_setFadeSpeed(uint8_t addr, uint8_t fadespeed);
     void BlinkM_setTimeAdjust(uint8_t addr, uint8_t adjust);
     void BlinkM_setRGB(uint8_t addr, uint8_t red, uint8_t grn, uint8_t blu);
     void BlinkM_fadeToRGB(uint8_t addr, uint8_t red, uint8_t grn, uint8_t blu);
    
     //void BlinkM_RGBslow();
     //void BlinkM_RGBfast();
     void BlinkM_Off(); 
     
     void PlayScript(uint8_t id);
     void BlinkM_writeScriptLine(uint8_t lineno,uint8_t cmd,uint8_t r,uint8_t g,uint8_t b);
     void BlinkM_SetScriptLength(uint8_t length,uint8_t repeats);
 
private:
    uint8_t _val;
    uint8_t _r;
    uint8_t _g;
    uint8_t _b;

};

#endif // __AP_HAL_AVR_NOTIFY_H__

