/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void read_control_switch()
{

// BUZZ TODO - DISABLE THIS FUNCTION WHEN WE NO LONGER REQUIRE MODE CHANGES VIA TX SWITCHES
byte switchPosition = readSwitch();
	if (oldSwitchPosition != switchPosition){

		//set_mode(flight_modes[switchPosition]);

                 // low pulse-width on channel 8 below 1230 means TERMNATE ! 
                if ( switchPosition == 0 ) {
                  set_mode(RTL); // or TERMINATE
                } else {
                 set_mode(AUTO); // GOOD !
                }
		oldSwitchPosition = switchPosition;
	}

}

//  0..5 for the 6 possible modes! 
static byte readSwitch(void){
	uint16_t pulsewidth = APM_RC.InputCh(g.flight_mode_channel - 1);
	if (pulsewidth > 1230 && pulsewidth <= 1360) 	return 1;
	if (pulsewidth > 1360 && pulsewidth <= 1490) 	return 2;
	if (pulsewidth > 1490 && pulsewidth <= 1620) 	return 3;
	if (pulsewidth > 1620 && pulsewidth <= 1749) 	return 4;	 
	if (pulsewidth >= 1750) 			return 5;	

        // BUZZ: if *claimed* pulse width == 900 EXACTLY on channel 8, then the PPM module IS LYING, because its LOST RADIO SIGNAL! 
        // in that case, we actually want to keep the model flying , so we fudge it to 1
        if (pulsewidth == 900) {  return 1;  } 
        
	return 0;  // if pulse < 1230.  ZERO MEANS RTL/TERMINATE 
}

static void reset_control_switch()
{

}

static void update_servo_switches()
{

}
