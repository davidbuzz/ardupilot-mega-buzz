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

#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <AP_HAL.h>
#include "Notify.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

void AVRNotify::BlinkM_stopScript(uint8_t addr)
{

  uint8_t data = 'c';
  hal.i2c->begin();
  hal.i2c->write(addr,1,&data); // adress, length, data 
  hal.i2c->end();
  
}

// Sets the speed of fading between colors.  
// Higher numbers means faster fading, 255 == instantaneous fading
void AVRNotify::BlinkM_setFadeSpeed(uint8_t addr, uint8_t fadespeed)
{

  hal.i2c->begin();
  uint8_t data = 'f';
  hal.i2c->write(addr,1,&data); // command
  hal.i2c->write(addr,1,&fadespeed); // speed
  hal.i2c->end();

}

void AVRNotify::BlinkM_setTimeAdjust(uint8_t addr, uint8_t adjust)
{

  hal.i2c->begin();
  uint8_t data = 't';
  hal.i2c->write(addr,1,&data); // command 
  hal.i2c->write(addr,1,&adjust); // adjust
  hal.i2c->end();

}

// Sets an RGB color immediately
void AVRNotify::BlinkM_setRGB(uint8_t addr, uint8_t red, uint8_t grn, uint8_t blu)
{

  hal.i2c->begin();
  uint8_t data = 'n';
  hal.i2c->write(addr,1,&data); // command 
  hal.i2c->write(addr,1,&red); // adress, length, data 
  hal.i2c->write(addr,1,&grn); // adress, length, data 
  hal.i2c->write(addr,1,&blu); // adress, length, data 
  hal.i2c->end();
}

// Fades to an RGB color
 void AVRNotify::BlinkM_fadeToRGB(uint8_t addr, uint8_t red, uint8_t grn, uint8_t blu)
{
  hal.i2c->begin();
  uint8_t data = 'c';
  hal.i2c->write(addr,1,&data); // adress, length, data 
  hal.i2c->write(addr,1,&red); // adress, length, data 
  hal.i2c->write(addr,1,&grn); // adress, length, data 
  hal.i2c->write(addr,1,&blu); // adress, length, data 
  hal.i2c->end();
}

/* 
void AVRNotify::BlinkM_RGBslow() { 
    
   //   hal.console->println("slow...");

//hal.console->print("RED ...");
 BlinkM_fadeToRGB(0, 255,0,0 );
   hal.scheduler->delay(2000);  
//hal.console->print("GREEN ...");
BlinkM_fadeToRGB(0, 0,255,0 );
   hal.scheduler->delay(2000);  
//hal.console->print("BLUE ...");
BlinkM_fadeToRGB(0, 0,0,255 );
   hal.scheduler->delay(2000);
//hal.console->println("");   
}
*/

void AVRNotify::BlinkM_Off() { 

 BlinkM_stopScript( 0 );
 BlinkM_setFadeSpeed(0,30);
 BlinkM_setRGB(0, 0,0,0 );
 //hal.scheduler->delay(1000); 

//hal.console->println("Off");   
}

void AVRNotify::PlayScript(uint8_t id) { 

 // BlinkM_stopScript( 0 );

  hal.i2c->begin();
  uint8_t data = 'p';
  hal.i2c->write(0,1,&data); // command
  hal.i2c->write(0,1,&id); // id
    uint8_t repeats = 255;
  hal.i2c->write(0,1,&repeats); // repeat count, 0 -= continuous? 
    uint8_t other = 0;
  hal.i2c->write(0,1,&other); // line of script to start on?
  hal.i2c->end();
   

//hal.console->println("Off");   
}

void AVRNotify::BlinkM_writeScriptLine(uint8_t lineno,uint8_t cmd,uint8_t r,uint8_t g,uint8_t b){// "line 1 of script is "green on"

  hal.i2c->begin();
  uint8_t data = 'W';
  hal.i2c->write(0,1,&data); // command
  data = 0;
  hal.i2c->write(0,1,&data); // script id = 0
  hal.i2c->write(0,1,&lineno);  // line number to write to 0-> 49
  data = 50;
  hal.i2c->write(0,1,&data);  //duration in ticks to persist 
  hal.i2c->write(0,1,&cmd);  // cmd to put in script   
  hal.i2c->write(0,1,&r);   // params for cmd in script 
  hal.i2c->write(0,1,&g); //..
  hal.i2c->write(0,1,&b); //..
  hal.i2c->end();
  

}
void AVRNotify::BlinkM_SetScriptLength(uint8_t length,uint8_t repeats) { // set script length to 2, with 255 repeats TODO does zero repeats mean infinite? 

  hal.i2c->begin();
  uint8_t data = 'L';
  hal.i2c->write(0,1,&data); // command
  data = 0;
  hal.i2c->write(0,1,&data); // script id = 0
  hal.i2c->write(0,1,&length);  // length
  hal.i2c->write(0,1,&repeats);  //repeats.? 
  hal.i2c->end();

}


/* 
void AVRNotify::BlinkM_RGBfast() { 

//hal.console->println("fast...");   
  for (int x=0 ; x< 5 ; x++ ) {  
//hal.console->print("R->G->B ...");
BlinkM_setRGB(0, 255,0,0 );
  hal.scheduler->delay(100);  
    BlinkM_setRGB(0, 0,255,0 );
   hal.scheduler->delay(100);  
    BlinkM_setRGB(0, 0,0,255 );
   hal.scheduler->delay(100);  
  }  
}
*/
 
void AVRNotify::set(uint8_t val) {
 _val = val;
}

// push color immediately to blinkm, and remember it. 
void AVRNotify::setrgb(uint8_t r, uint8_t g, uint8_t b){
     _r = r; 
     _g = g;
     _b = b;  
     BlinkM_setRGB( 0,  r, g , b ) ; 
}


void AVRNotify::notify(uint8_t n) { 
    
  // same as set(), to start with
   _val = n;  
  
  // then do other stuff too: 
  switch( n) { 
    
    case POWERON: 

        // turn all lights off at POWERON
        BlinkM_Off();
  
    break; 
    case RGB_FAST:  // siren-like-attention-getter ( very fast, 1.5sec => 15 transitions) 
        //BlinkM_RGBfast();  //TODO THE DELAYS IN HERE ARE BAD. WE SHOULD PUSH A SCRIPT TO THE BLINKM FOR THIS.   
        hal.console->println("RGB_FAST");
        BlinkM_stopScript(0);
        BlinkM_setFadeSpeed(0,30);  // range from 1 ( slowest fase ) ->255 ( fastest fade )  
        BlinkM_setTimeAdjust(0,(uint8_t)0); // range from -127 ( fastest )  ->127 ( slowest ) 
        PlayScript(1); // the pre-canned RGB script. 
    break;
    case RGB_SLOW:  // slow and docile at ~1/6th Hz ( 2 secs => 1 transition ) 
       // BlinkM_RGBslow();  //TODO THE DELAYS IN HERE ARE BAD. WE SHOULD PUSH A SCRIPT TO THE BLINKM FOR THIS. 
        hal.console->println("RGB_SLOW");
        BlinkM_stopScript(0);
        BlinkM_setFadeSpeed(0,10);  // range from 1 ( slowest fade ) ->255 ( fastest fade )  
        BlinkM_setTimeAdjust(0,127); // range from -127 ( fastest )  ->127 ( slowest ) 
        PlayScript(1); // the pre-canned RGB script. 
    break;
    case RED_SOLID_MOTORS_ARMED:
        hal.console->println("RED_SOLID_MOTORS_ARMED");
        BlinkM_stopScript(0);
        setrgb(255,_g,_b);
    break;
    case RED_FLASH_MOTORS_DISARMED:
        hal.console->println("RED_FLASH_MOTORS_DISARMED");
        BlinkM_stopScript(0);
        BlinkM_setFadeSpeed(0,128);  // range from 1 ( slowest fade ) ->255 ( fastest fade )  
        BlinkM_setTimeAdjust(0,0); // range from -127 ( fastest )  ->127 ( slowest ) 
        PlayScript(3); // the pre-canned R flash script. 
    break;
    case RED_GREEN_FAST_FLASH_CALIBRATE_GYRO:  //A = green, 
        //TODO, using eeprom code and script id zero.
        hal.console->println("RED_GREEN_FAST_FLASH_CALIBRATE_GYRO");
        BlinkM_stopScript(0);
        BlinkM_writeScriptLine(0,'c',255,0,0);// line, cmd, arg, arg, arg , or "line zero of script is "red on"
        BlinkM_writeScriptLine(1,'c',0,255,0);// "line 1 of script is "green on"
        BlinkM_SetScriptLength(2,255); // set script length to 2, with 255 repeats TODO does zero repeats mean infinite? 
        
    break;
    case YELLOW_FLASH_CALIBRATING:
        hal.console->println("YELLOW_FLASH_CALIBRATING");
        BlinkM_stopScript(0);
        BlinkM_setFadeSpeed(0,255);  // range from 1 ( slowest fase ) ->255 ( fastest fade )  
        BlinkM_setTimeAdjust(0,(uint8_t)-127); // range from -127 ( fastest )  ->127 ( slowest ) 
        PlayScript(8); // the pre-canned yellow flash script. 
    break;
    case YELLOW_SOLID_READY_EXCEPT_FOR_GPS_LOCK:   // B
        hal.console->println("YELLOW_SOLID_READY_EXCEPT_FOR_GPS_LOCK");
        BlinkM_stopScript(0);
        setrgb(255,255,_b);  // yellow = 100% of red and green.
    break;
    case BLUE_FLASH_NO_GPS_LOCK:
        hal.console->println("BLUE_FLASH_NO_GPS_LOCK");
        BlinkM_stopScript(0);
        BlinkM_setFadeSpeed(0,128);  // range from 1 ( slowest fase ) ->255 ( fastest fade )  
        BlinkM_setTimeAdjust(0,0); // range from -127 ( fastest )  ->127 ( slowest ) 
        PlayScript(5); // the pre-canned blue flash script. 
    break;
    case BLUE_SOLID_GPS_LOCK_OK:   //C
        hal.console->println("BLUE_SOLID_GPS_LOCK_OK");
        BlinkM_stopScript(0);
        setrgb(_r,_g,255);     
    break;             
    case CALIBRATION_ROUTINE:
        hal.console->println("CALIBRATION_ROUTINE");
        setrgb(0,0,0); 
        PlayScript(0);
    break; 
    case PURPLE: // for future use. 
        hal.console->println("PURPLE");
        BlinkM_stopScript(0);
        setrgb(255,_g,255);     // red+blue = purple. 
    break;
    default: 
    
    break;
  }   
    
} 
    

#endif
