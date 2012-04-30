// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
	We will determine later if we are actually on the ground and process a
	ground start in that case.

*****************************************************************************/

#if CLI_ENABLED == ENABLED

#endif // CLI_ENABLED

static void init_ardupilot()
{
#if TELEMETRY_ON_SERIAL0 == 1   // we might put telemetry to Serial0 instead of Serial3, even without the MUX, 
        delay( 1000 ) ; 
#endif 
	// Console serial port
	//
	// The console port buffers are defined to be sufficiently large to support
	// the console's use as a logging device, optionally as the GPS port when
	// GPS_PROTOCOL_IMU is selected, and as the telemetry port.
	//
	// XXX This could be optimised to reduce the buffer sizes in the cases
	// where they are not otherwise required.
	//
	Serial.begin(SERIAL0_BAUD, 128, 128);

	// GPS serial port.
	//
	// XXX currently the EM406 (SiRF receiver) is nominally configured
	// at 57600, however it's not been supported to date.  We should
	// probably standardise on 38400.
	//
	// XXX the 128 byte receive buffer may be too small for NMEA, depending
	// on the message set configured.
	//
    // standard gps running
    Serial1.begin(38400, 128, 16);

	Serial.printf_P(PSTR("\n\nInit " THISFIRMWARE
						 "\n\nFree RAM: %u\n"),
                    memcheck_available_memory());

	//
	// Initialize the ISR registry.
	//
    isr_registry.init();
    //
	// Initialize the timer scheduler to use the ISR registry.
	//

    timer_scheduler.init( & isr_registry );
	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
    load_parameters();

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
   // g.num_resets.set_and_save(g.num_resets+1);

	// init the GCS
	gcs0.init(&Serial);

#if USB_MUX_PIN > 0
    if (!usb_connected) {
        // we are not connected via USB, re-init UART0 with right
        // baud rate
        Serial.begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
    }
#endif

// wack serial zero into the baud rate that we are supposed to use in Serial3
#if TELEMETRY_ON_SERIAL0 == 1 
         Serial.begin(57600, 128, 128); //TODO DONT HARDCODE THIS, BUT DONT MAKE IT SERIAL0_BAUD OR SERIAL3_BAUD it may clash! 
#endif 

#if SERIAL3_INIT == 1 
    // we have a 2nd serial port, possibly for telemetry, or maybe for other stuff 
        Serial3.begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
#endif
#if TELEMETRY_ON_SERIAL0 != 1  && SERIAL3_INIT == 1 
     // ok, so we're doing hte classic APM1 thing for telemetry.. 
	gcs3.init(&Serial3);
#endif


	mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
#endif
	pinMode(C_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(A_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(B_LED_PIN, OUTPUT);			// GPS status LED
       #if EXTRA_GPS_DEBUG == 1
          digitalWrite(A_LED_PIN, 1 );
          delay(1000);        
          #endif  
          

	// Do GPS init
	g_gps = &g_gps_driver;
	g_gps->init();			// GPS Initialization
        g_gps->callback = mavlink_delay;
        g_gps->update();
        	
          #if EXTRA_GPS_DEBUG == 1
          digitalWrite(A_LED_PIN, 0 );
          delay(500);        
          digitalWrite(B_LED_PIN, 1 );
          delay(1000);        
          #endif 
          
        #if EXTRA_GPS == ENABLED
  	// Do other GPS init
	g_gps2 = &g_gps_driver2;
	g_gps2->init();			// EXTRA GPS Initialization
        g_gps2->callback = mavlink_delay;
        g_gps2->update();	
        #endif
        
          #if EXTRA_GPS_DEBUG == 1
          digitalWrite(B_LED_PIN, 0 );
          #endif

 
	//mavlink_system.sysid = MAV_SYSTEM_ID;				// Using g.sysid_this_mav
	mavlink_system.compid = 1;	//MAV_COMP_ID_IMU;   // We do not check for comp id
	mavlink_system.type = MAV_FIXED_WING;

	rc_override_active = APM_RC.setHIL(rc_override);		// Set initial values for no override

    RC_Channel::set_apm_rc( &APM_RC ); // Provide reference to RC outputs.
	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

#if SLIDE_SWITCH_PIN > 0
	pinMode(SLIDE_SWITCH_PIN, INPUT);	// To enter interactive mode
#endif
#if CONFIG_PUSHBUTTON == ENABLED
	pinMode(PUSHBUTTON_PIN, INPUT);		// unused
#endif
#if CONFIG_RELAY == ENABLED
	DDRL |= B00000100;					// Set Port L, pin 2 to output for the relay
#endif

#if FENCE_TRIGGERED_PIN > 0
    pinMode(FENCE_TRIGGERED_PIN, OUTPUT);
    digitalWrite(FENCE_TRIGGERED_PIN, LOW);
#endif

    /*
      setup the 'main loop is dead' check. Note that this relies on
      the RC library being initialised.
     */
  //  timer_scheduler.set_failsafe(failsafe_check);



	// read in the flight switches
	update_servo_switches();

		// Perform an air start and get back to flying
		gcs_send_text_P(SEVERITY_LOW,PSTR("<init_ardupilot> AIR START"));


		// This delay is important for the APM_RC library to work.
		// We need some time for the comm between the 328 and 1280 to be established.
		int old_pulse = 0;
		while (millis()<=1000 && (abs(old_pulse - APM_RC.InputCh(g.flight_mode_channel)) > 5 ||
					APM_RC.InputCh(g.flight_mode_channel) == 1000 ||
					APM_RC.InputCh(g.flight_mode_channel) == 1200)) {
			old_pulse = APM_RC.InputCh(g.flight_mode_channel);
			delay(25);
		}
		GPS_enabled = false;
		g_gps->update();
		if (g_gps->status() != 0 || HIL_MODE != HIL_MODE_DISABLED)	GPS_enabled = true;

		if (g.log_bitmask & MASK_LOG_CMD)
			Log_Write_Startup(TYPE_AIRSTART_MSG);
		reload_commands_airstart();		// Get set to resume AUTO from where we left off



        digitalWrite(B_LED_PIN, HIGH);		// Set LED B high to indicate IMU ready
	digitalWrite(A_LED_PIN, LOW);
	digitalWrite(C_LED_PIN, LOW);

        demo_servos(1);



    // BUZZ DEFAULT TO AUTO 
    set_mode(AUTO);

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();
}


#if EXTRA_GPS == ENABLED
 void use_best_gps(void){
  
   static int _active = 1;   // default to primary GPS on Serial1 first  
   #ifdef  EXTRA_GPS_DEBUG  
   static int _which = 0; // primary GPS defaults to A LED being OFF. alternate GPS turns this LED on.! 
   #endif
   
   int G1 = g_gps->num_sats; // currently active
   int G3 = g_gps2->num_sats;  // currently inactive/backup
     
    // quick, decide which GPS is better? 
    // for now, we use the number of sattelites, but we *could* also use GDOP values 
    // or similar, if the appropriate module/s supply this info
    if ( G3 > G1 ) { 
      
    #ifdef  EXTRA_GPS_DEBUG  
      if ( _which == 0 ) {  _which=1; _active=3 ; } else { _which=0;_active = 1 ; } // which serial are we switching to... Serial1 or Serial3? 
      Serial.print("GPS FLIP! Now using Serial");
      Serial.print(_active);
      Serial.print(" ( Sats: ");
      Serial.print((int)G3);
      Serial.println(" )");
   #endif         
           
      //switch the pointers for g_gps, and g_gps2, so we go to the other GPS! 
      g_gpscurrent = g_gps; 
      g_gps = g_gps2;
      g_gps2 = g_gpscurrent;
           
    #ifdef EXTRA_GPS_DEBUG  
      //indicate the GPS toggle state with an extra LED 
      digitalWrite(A_LED_PIN, _which );
    #endif  
        } else {
    #ifdef  EXTRA_GPS_DEBUG  
      Serial.print("GPS Sats? active: ");
      Serial.print( (int)G1);
      Serial.print(" inactive: ");
      Serial.println( (int)G3);
    #endif
        }  
}
#endif

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
static void startup_ground(void)
{
  
}

static void set_mode(byte mode)
{
	if(control_mode == mode){
		// don't switch modes if we are already in the correct mode.
		return;
	}
	control_mode = mode;
	crash_timer = 0;


}


static void check_short_failsafe()
{
 
}



static void update_GPS_light(void)
{
	// GPS LED on if we have a fix or Blink GPS LED if we are receiving data
	// ---------------------------------------------------------------------
	switch (g_gps->status()) {
		case(2):
			digitalWrite(C_LED_PIN, HIGH);  //Turn LED C on when gps has valid fix.
			break;

		case(1):
			if (g_gps->valid_read == true){
				GPS_light = !GPS_light; // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
				if (GPS_light){
					digitalWrite(C_LED_PIN, LOW);
				} else {
					digitalWrite(C_LED_PIN, HIGH);
				}
				g_gps->valid_read = false;
			}
			break;

		default:
			digitalWrite(C_LED_PIN, LOW);
			break;
	}
}


static void resetPerfData(void) {
	mainLoop_count 			= 0;
	G_Dt_max 				= 0;

	gps_fix_count 			= 0;
	pmTest1					= 0;
	perf_mon_timer 			= millis();
}


/*
  map from a 8 bit EEPROM baud rate to a real baud rate
 */
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud)
{
    switch (rate) {
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 111:  return 111100;
    case 115:  return 115200;
    }
    Serial.println_P(PSTR("Invalid SERIAL3_BAUD"));
    return default_baud;
}
