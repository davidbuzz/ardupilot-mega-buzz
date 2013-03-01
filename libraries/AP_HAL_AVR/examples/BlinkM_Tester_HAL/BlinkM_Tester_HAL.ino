
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
//#include <AP_HAL_PX4.h>
//#include <AP_HAL_Empty.h>
//#include <AP_Buffer.h>
//#include <Filter.h>
//#include <AP_Baro.h>

//const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

//#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
//#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
//const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
//#endif


//    hal.console->println("No I2C devices found");

    

//  hal.scheduler->delay(100); // wait a bit for things to stabilize


//
static void BlinkM_stopScript(uint8_t addr)
{
  /* Wire.beginTransmission(addr);
  Wire.write('o');
  Wire.endTransmission();
  */
  uint8_t data = 'c';
  hal.i2c->begin();
  uint8_t stat = hal.i2c->write(addr,1,&data); // adress, length, data 
  hal.i2c->end();
  
}

// Sets the speed of fading between colors.  
// Higher numbers means faster fading, 255 == instantaneous fading
static void BlinkM_setFadeSpeed(uint8_t addr, uint8_t fadespeed)
{
  /* Wire.beginTransmission(addr);
  Wire.write('f');
  Wire.write(fadespeed);
  Wire.endTransmission();  
  */
  hal.i2c->begin();
  uint8_t data = 'f';
  uint8_t stat = hal.i2c->write(addr,1,&data); // adress, length, data 
  stat = hal.i2c->write(addr,1,&fadespeed); // adress, length, data 
  hal.i2c->end();

}

// Sets an RGB color immediately
static void BlinkM_setRGB(uint8_t addr, uint8_t red, uint8_t grn, uint8_t blu)
{
  /* Wire.beginTransmission(addr);
  Wire.write('n');
  Wire.write(red);
  Wire.write(grn);
  Wire.write(blu);
  Wire.endTransmission();
  */
  hal.i2c->begin();
  uint8_t data = 'n';
  uint8_t stat = hal.i2c->write(addr,1,&data); // adress, length, data 
  stat = hal.i2c->write(addr,1,&red); // adress, length, data 
  stat = hal.i2c->write(addr,1,&grn); // adress, length, data 
  stat = hal.i2c->write(addr,1,&blu); // adress, length, data 
  hal.i2c->end();
}

// Fades to an RGB color
 static void BlinkM_fadeToRGB(uint8_t addr, uint8_t red, uint8_t grn, uint8_t blu)
{
  /* 
  Wire.beginTransmission(addr);
  Wire.write('c');
  Wire.write(red);
  Wire.write(grn);
  Wire.write(blu);
  Wire.endTransmission();
  */
  hal.i2c->begin();
  uint8_t data = 'c';
  uint8_t stat = hal.i2c->write(addr,1,&data); // adress, length, data 
  stat = hal.i2c->write(addr,1,&red); // adress, length, data 
  stat = hal.i2c->write(addr,1,&grn); // adress, length, data 
  stat = hal.i2c->write(addr,1,&blu); // adress, length, data 
  hal.i2c->end();
}


// arduino setup func
void setup()
{
 
 // Wire.begin();   
 hal.console->println("BlinkM in HAL");


//  hal.scheduler->delay(100); // wait a bit for things to stabilize
  
  // turn ll lights off.
  BlinkM_stopScript( 0 );
  BlinkM_setFadeSpeed(0,30);
  
  BlinkM_setRGB(0, 0,0,0 );

  //Serial.begin(19200);

}

// arduino loop func
void loop()
{

   hal.console->println("slow...");

hal.console->print("RED ...");
 BlinkM_fadeToRGB(0, 255,0,0 );
   hal.scheduler->delay(2000);  
hal.console->print("GREEN ...");
BlinkM_fadeToRGB(0, 0,255,0 );
   hal.scheduler->delay(2000);  
hal.console->print("BLUE ...");
BlinkM_fadeToRGB(0, 0,0,255 );
   hal.scheduler->delay(2000);
   
hal.console->println("");   

hal.console->println("fast...");   
  for (int x=0 ; x< 5 ; x++ ) {  
hal.console->print("R->G->B ...");
BlinkM_setRGB(0, 255,0,0 );
  hal.scheduler->delay(100);  
    BlinkM_setRGB(0, 0,255,0 );
   hal.scheduler->delay(100);  
    BlinkM_setRGB(0, 0,0,255 );
   hal.scheduler->delay(100);  
  }
   // off
 BlinkM_setRGB(0, 0,0,0 );
 hal.scheduler->delay(1000); 

hal.console->println("");
hal.console->println("");
  
}


AP_HAL_MAIN();

