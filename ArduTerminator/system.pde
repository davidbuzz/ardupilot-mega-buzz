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
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
	if (!g.format_version.load()) {

        Serial.println_P(PSTR("\nEEPROM blank - resetting all parameters to defaults...\n"));
        delay(100); // wait for serial msg to flush

		AP_Var::erase_all();

		// save the current format version
		g.format_version.set_and_save(Parameters::k_format_version);

    } else if (g.format_version != Parameters::k_format_version) {

		Serial.printf_P(PSTR("\n\nEEPROM format version %d not compatible with this firmware (requires %d)"
		                     "\n\nForcing complete parameter reset..."),
		                     g.format_version.get(), Parameters::k_format_version);
		delay(100); // wait for serial msg to flush

		// erase all parameters
		AP_Var::erase_all();

		// save the new format version
		g.format_version.set_and_save(Parameters::k_format_version);

		Serial.println_P(PSTR("done."));
	} else {
	    unsigned long before = micros();
	    // Load all auto-loaded EEPROM variables
	    AP_Var::load_all();

	    Serial.printf_P(PSTR("load_all took %luus\n"), micros() - before);
	    Serial.printf_P(PSTR("using %u bytes of memory (%u resets)\n"), 
                        AP_Var::get_memory_use(), (unsigned)g.num_resets);
	}

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

	// Telemetry port.
	//
	// Not used if telemetry is going to the console.
	//
	// XXX for unidirectional protocols, we could (should) minimize
	// the receive buffer, and the transmit buffer could also be
	// shrunk for protocols that don't send large messages.
	//
	Serial3.begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);

	mavlink_system.sysid = g.sysid_this_mav;


#if LOGGING_ENABLED == ENABLED
#endif

	// Do GPS init
	g_gps = &g_gps_driver;
	g_gps->init();			// GPS Initialization
    g_gps->callback = mavlink_delay;

	// init the GCS
	gcs0.init(&Serial);
	gcs3.init(&Serial3);

	//mavlink_system.sysid = MAV_SYSTEM_ID;				// Using g.sysid_this_mav
	mavlink_system.compid = 1;	//MAV_COMP_ID_IMU;   // We do not check for comp id
	mavlink_system.type = MAV_FIXED_WING;

	rc_override_active = APM_RC.setHIL(rc_override);		// Set initial values for no override

	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

	pinMode(C_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(A_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(B_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(SLIDE_SWITCH_PIN, INPUT);	// To enter interactive mode
	pinMode(PUSHBUTTON_PIN, INPUT);		// unused
	DDRL |= B00000100;					// Set Port L, pin 2 to output for the relay


	if(g.log_bitmask != 0){
		//	TODO - Here we will check  on the length of the last log
		//  We don't want to create a bunch of little logs due to powering on and off
		byte last_log_num = get_num_logs();
		start_new_log(last_log_num);
	}

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
