// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

//
// Example code for the FastSerial driver.
//
// This code is placed into the public domain.
//

//
// Include the FastSerial library header.
//
// Note that this causes the standard Arduino Serial* driver to be
// disabled.
//
#include <FastSerial.h>
#include <AP_Common.h>
/* Not required by this sketch, but required by AP_Common */
#include <AP_Math.h>


#if __GNUC__ == 4 && __GNUC_MINOR__ == 5
#warning avr-gcc 4.5.x is known to have a bug with FastSerialPort2 and 3
/* avr-gcc 4.5.3 will leave off the USART2 and USART3 vectors from the vector
 * table. It will crash this example sketch at millis == 2000.
 * See http://gcc.gnu.org/bugzilla/show_bug.cgi?id=47696 for info.
 * Known to work:
 * avr-gcc 4.3.2 as shipped by Arduino IDE on Windows.
 * avr-gcc 4.4.2 does not have this bug (unknown status otherwise)
 * avr-gcc 4.6.2 has other issues that need to be worked out
 *  -pch 15 October 2012
 */
#endif
//
// Create a FastSerial driver that looks just like the stock Arduino
// driver, on each serial port.
//
FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort2(Serial2);
FastSerialPort3(Serial3);


void setup(void)
{
        //
        // Set the speed for our replacement serial port.
        //
        Serial.begin(115200);
        hal.uart1->begin(115200);
        hal.uart2->begin(115200);
        hal.uart3->begin(115200);

        do {
          Serial.print("hello serial0 millis: ");
          Serial.println(millis(), DEC);
        } while (millis() < 1000);
        
        do {
          hal.uart1->println("hello serial1");
          Serial.print("hello serial0 millis: ");
          Serial.println(millis(), DEC);
        } while (millis() < 2000);

        do {
          hal.uart2->println("hello serial2");
          Serial.print("hello serial0 millis: ");
          Serial.println(millis(), DEC);
        } while (millis() < 3000);

        do {
          hal.uart3->println("hello serial3");
          Serial.print("hello serial0 millis: ");
          Serial.println(millis(), DEC);
        } while (millis() < 4000);

}

void
loop(void)
{
    int    c;

    //
    // Perform a simple loopback operation on each port.
    //
    c = Serial.read();
    if (-1 != c)
        Serial.write(c);

    c = hal.uart1->read();
    if (-1 != c)
        hal.uart1->write(c);

    c = hal.uart2->read();
    if (-1 != c)
        hal.uart2->write(c);
    
    c = hal.uart3->read();
    if (-1 != c)
        hal.uart3->write(c);
}
