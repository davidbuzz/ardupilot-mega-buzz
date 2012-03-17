
static void stabilize()
{
}


static void crash_checker()
{
	
}


static void calc_throttle()
{
  if (g.airspeed_enabled == false) {

		g.channel_throttle.servo_out = constrain(g.channel_throttle.servo_out, g.throttle_min.get(), g.throttle_max.get());
	} else {
		// throttle control with airspeed compensation
		// -------------------------------------------
		energy_error = airspeed_energy_error + (float)altitude_error * 0.098f;

		// positive energy errors make the throttle go higher
		g.channel_throttle.servo_out = g.throttle_cruise + g.pidTeThrottle.get_pid(energy_error, dTnav);
		g.channel_throttle.servo_out += (g.channel_pitch.servo_out * g.kff_pitch_to_throttle);

		g.channel_throttle.servo_out = constrain(g.channel_throttle.servo_out,
			g.throttle_min.get(), g.throttle_max.get());			// TODO - resolve why "saved" is used here versus "current"
	}

}



#define YAW_DAMPENER 0


/*****************************************
 * Throttle slew limit
 *****************************************/
static void throttle_slew_limit()
{

}


// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
static void reset_I(void)
{
	g.pidNavRoll.reset_I();
	g.pidNavPitchAirspeed.reset_I();
	g.pidNavPitchAltitude.reset_I();
	g.pidTeThrottle.reset_I();
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/

//  also see read_radio() for the bit that needs to happen first 
static void set_servos(void)
{
	int flapSpeedSource = 0;


// from radio, to servos
//radio_in -> radio_out -> servo_out



// THIS IS JUST A BLANK PASS_THROUGH OF THE VALUES ALREADY IN *.radio_in from read_radio , no re-read needed! 
  
  if ( control_mode == AUTO ) { 
  	  		g.channel_roll.radio_out 		= g.channel_roll.radio_in;
  			g.channel_pitch.radio_out 		= g.channel_pitch.radio_in;
  		g.channel_throttle.radio_out 	= g.channel_throttle.radio_in;
  		g.channel_rudder.radio_out 		= g.channel_rudder.radio_in;
  
  
  			g.rc_5.radio_out 		= g.rc_5.radio_in;
  			g.rc_6.radio_out 		= g.rc_6.radio_in;
  		g.rc_7.radio_out 	= g.rc_7.radio_in;
  		g.rc_8.radio_out 		= g.rc_8.radio_in;
  }  
  
  // don't pass input to output in case of TERMINATE ! 
  if ( ( control_mode == RTL  ) || (   control_mode == TERMINATE ) )  { 
    
    // max-out all output chanels except throttle !   
    // we can figure out if we need to reverse any later! 
    // TODO REVERSING? 
    
  	  		g.channel_roll.radio_out 		=  2000; // g.channel_roll.radio_in;
  			g.channel_pitch.radio_out 		= 2000;// g.channel_pitch.radio_in;
  		        g.channel_throttle.radio_out 	= 0;// g.channel_throttle.radio_in;
  		        g.channel_rudder.radio_out 		= 2000;//g.channel_rudder.radio_in;
  
  
  			g.rc_5.radio_out 		= 2000; //g.rc_5.radio_in;
  			g.rc_6.radio_out 		= 2000; //g.rc_6.radio_in;
  		        g.rc_7.radio_out 	        = 2000;             //DROP = 7 
  		        g.rc_8.radio_out 		= 2000; //g.rc_8.radio_in;
  
  } 





	// send values to the PWM timers for output
	// ----------------------------------------
 	APM_RC.OutputCh(CH_1, g.channel_roll.radio_out); // send to Servos
	APM_RC.OutputCh(CH_2, g.channel_pitch.radio_out); // send to Servos
	APM_RC.OutputCh(CH_3, g.channel_throttle.radio_out); // send to Servos
	APM_RC.OutputCh(CH_4, g.channel_rudder.radio_out); // send to Servos

	// Route configurable aux. functions to their respective servos
 	g.rc_5.output_ch(CH_5);
	g.rc_6.output_ch(CH_6);
	g.rc_7.output_ch(CH_7);
	g.rc_8.output_ch(CH_8);

     APM_RC.OutputCh(CH_5, 	g.rc_5.radio_out);
     APM_RC.OutputCh(CH_6, 	g.rc_6.radio_out);
     APM_RC.OutputCh(CH_7,   g.rc_7.radio_out);
     APM_RC.OutputCh(CH_8,   g.rc_8.radio_out);

 

}

static void demo_servos(byte i) {


}

void terminate_servos () { 
  
		APM_RC.OutputCh(3, 900); // throttle to OFF
		mavlink_delay(400);
		APM_RC.OutputCh(5, 2000); // chan 5 ( bottle drop?) to FULL
		APM_RC.OutputCh(7, 2000); // chan 7 ( parachute?) to FULL
		mavlink_delay(400);
		APM_RC.OutputCh(2, 2000);  //RUDD? 
		APM_RC.OutputCh(1, 2000);  //ELE
		APM_RC.OutputCh(4, 2000);  //AIL?
		//APM_RC.OutputCh(1, 900);  //ANTYTHING ELSE?  
		mavlink_delay(400);
} 

