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

class AP_HAL::Notify {
public:
    virtual void set(uint8_t val) = 0;
    
    virtual void setrgb(uint8_t r, uint8_t g, uint8_t b); 
        
    virtual void notify(uint8_t n); 

   virtual void BlinkM_stopScript(uint8_t addr);
   virtual void BlinkM_setFadeSpeed(uint8_t addr, uint8_t fadespeed);
   virtual void BlinkM_setTimeAdjust(uint8_t addr, uint8_t adjust) ;
   virtual void BlinkM_setRGB(uint8_t addr, uint8_t red, uint8_t grn, uint8_t blu);
   virtual void BlinkM_fadeToRGB(uint8_t addr, uint8_t red, uint8_t grn, uint8_t blu);
   
   //virtual void BlinkM_RGBslow();
   //virtual void BlinkM_RGBfast();
   virtual void BlinkM_Off(); 

   virtual void PlayScript(uint8_t id);
   virtual void BlinkM_writeScriptLine(uint8_t lineno,uint8_t cmd,uint8_t r,uint8_t g,uint8_t b);
   virtual void BlinkM_SetScriptLength(uint8_t length,uint8_t repeats);
     
};

#endif // __AP_HAL_NOTIFY_H__

