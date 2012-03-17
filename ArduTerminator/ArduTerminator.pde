/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduTerminator Code@V2.251/Libraries@2.27"
/*
  ArduTerminator, based apon the excellent work of the authors of ArduPlane and ArduPilot, see below for their details
  Author: Buzz aka DavidBuzz email: davidbuzz@gmail.com
  Date: Nov 2011 
  
  NOTES FOR THE WARY:
  
  * only valid flight modes are AUTO and RTL.   everything else is AUTO.    'TERMINATE' is a synonym for RTL, but GCS doesn't understand it, so we use RTL. 
  * if channel 8 < 1230 , then we also flight-terminate, but not if the channel is disconnected or empty ( in which case it reads 900 exactly ) 
  * unterminateing is as simple as flicking the switch on channel 8, OR uploading a new set of wapoints, or clicking 'Auto' or 'Manual' in APM Planner.
  *  termination "boundary" is a certain "thickness"  ( approx 10 ft ) , if you come toward boundary from either side it will terminate. 
  *  GPS drift is a bitch, leave yourslf plenty of room , or you'll accidentially terminate! 
  *  at terminate, all channels go to full  (2000) except throttle which goes to minimum ( 0 ) 
  *  when you connect to the termination module via mavlink, it reports it's sysid of 2 , so you can tell it's not the navigational APM. ( with a sysid of 1 ) 
  * 
*/

/*
ArduPilot and ArduPlane Notice/s: 
Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short
Thanks to:  Chris Anderson, HappyKillMore, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi
Please contribute your ideas!

This firmware is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
*/

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////


// AVR runtime
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>
#include "vectors.h"

// Libraries
#include <FastSerial.h>
#include <AP_Common.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <AP_GPS.h>         // ArduPilot GPS library
#include <Wire.h>			// Arduino I2C lib
#include <SPI.h>			// Arduino SPI lib
#include <DataFlash.h>      // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
//#include <APM_BMP085.h>     // ArduPilot Mega BMP085 Library
//#include <AP_Baro.h>        // ArduPilot barometer library
//#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_IMU.h>         // ArduPilot Mega IMU Library
#include <AP_DCM.h>         // ArduPilot Mega DCM Library
//#include <AP_Baro.h>        // ArduPilot barometer library

#include <PID.h>            // PID library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>	// Range finder library
#include <ModeFilter.h>
#include <AP_Relay.h>       // APM relay
#include <AP_Mount.h>		// Camera/Antenna mount
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <memcheck.h>

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
#include "Parameters.h"
#include "GCS.h"

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
//
// Note that FastSerial port buffers are allocated at ::begin time,
// so there is not much of a penalty to defining ports that we don't
// use.
//
FastSerialPort0(Serial);        // FTDI/console
FastSerialPort1(Serial1);       // GPS port
FastSerialPort3(Serial3);       // Telemetry port



// which RC library R we using? 
APM_RC_APM1 APM_RC;

// which dataflash library r we using? 
 DataFlash_APM1   DataFlash;

////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
Arduino_Mega_ISR_Registry isr_registry;


static Parameters      g;


////////////////////////////////////////////////////////////////////////////////
// prototypes
static void update_events(void);


////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode.  Real sensors are used.
// - HIL Attitude mode.  Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode.  Synthetic sensors are configured that
//   supply data from the simulation.
//

// All GPS access should be through this pointer.
static GPS         *g_gps;

// flight modes convenience array, there are 6 potential modes, we just point to the first of them! 
static AP_Int8		*flight_modes = &g.flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

// real sensors
static AP_ADC_ADS7844          adc;
//static AP_Baro_BMP085          barometer(false);
//static APM_BMP085_Class        barometer;
//static AP_Compass_HMC5843      compass(Parameters::k_param_compass);

#if GPS_PROTOCOL == GPS_PROTOCOL_MTK16
AP_GPS_MTK16    g_gps_driver(&Serial1);

#elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
//AP_GPS_None     g_gps_driver(NULL);
#else
 #error Unrecognised GPS_PROTOCOL setting.
#endif // GPS PROTOCOL

#elif HIL_MODE == HIL_MODE_SENSORS 

#elif HIL_MODE == HIL_MODE_ATTITUDE     /*    BUZZ:  THIS IS THE ONLY MODE WE USE FOR TESTING THIS IN HIL */
AP_DCM_HIL              dcm;
AP_GPS_HIL              g_gps_driver(NULL);


#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
//
GCS_MAVLINK	gcs0(Parameters::k_param_streamrates_port0);
GCS_MAVLINK	gcs3(Parameters::k_param_streamrates_port3);

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//
ModeFilter sonar_mode_filter;

#if SONAR_TYPE == MAX_SONAR_XL
//	AP_RangeFinder_MaxsonarXL sonar(&adc, &sonar_mode_filter);//(SONAR_PORT, &adc);
#elif SONAR_TYPE == MAX_SONAR_LV
	// XXX honestly I think these output the same values
	// If someone knows, can they confirm it?
//	AP_RangeFinder_MaxsonarXL sonar(&adc, &sonar_mode_filter);//(SONAR_PORT, &adc);
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

byte    control_mode        = INITIALISING;
byte    oldSwitchPosition;              // for remembering the control mode switch
bool    inverted_flight     = false;

static const char *comma = ",";

static const char* flight_mode_strings[] = {
	"Manual",
	"Circle",
	"Stabilize",
	"",
	"",
	"FBW_A",
	"FBW_B",
	"",
	"",
	"",
	"Auto",
	"RTL",
	"Loiter",
	"Takeoff",
	"Land",
        "Terminate"};



// Failsafe
// --------
static int 	failsafe;					// track which type of failsafe is being processed
static bool 	ch3_failsafe;
static byte    crash_timer;

// Radio
// -----
static uint16_t elevon1_trim  = 1500; 	// TODO: handle in EEProm
static uint16_t elevon2_trim  = 1500;
static uint16_t ch1_temp      = 1500;     // Used for elevon mixing
static uint16_t ch2_temp  	= 1500;
static int16_t rc_override[8] = {0,0,0,0,0,0,0,0};
static bool rc_override_active = false;
static uint32_t rc_override_fs_timer = 0;
static uint32_t ch3_failsafe_timer = 0;

// for elevons radio_in[CH_ROLL] and radio_in[CH_PITCH] are equivalent aileron and elevator, not left and right elevon

// LED output
// ----------
static bool GPS_light;							// status of the GPS light

// GPS variables
// -------------
static const 	float t7			= 10000000.0;	// used to scale GPS values for EEPROM storage
static float 	scaleLongUp			= 1;			// used to reverse longitude scaling
static float 	scaleLongDown 		= 1;			// used to reverse longitude scaling
static byte 	ground_start_count	= 5;			// have we achieved first lock and set Home?
static int     ground_start_avg;					// 5 samples to avg speed for ground start
static bool	GPS_enabled 	= false;			// used to quit "looking" for gps with auto-detect if none present

// Location & Navigation
// ---------------------
const	float radius_of_earth 	= 6378100;	// meters
const	float gravity 			= 9.81;		// meters/ sec^2
static long	nav_bearing;						// deg * 100 : 0 to 360 current desired bearing to navigate
static long	target_bearing;						// deg * 100 : 0 to 360 location of the plane to the target
static long	bearing_A;
static long	bearing_B;
static long	bearing_C;						
static long	crosstrack_bearing;					// deg * 100 : 0 to 360 desired angle of plane to target
static float	nav_gain_scaler 		= 1;		// Gain scaling for headwind/tailwind TODO: why does this variable need to be initialized to 1?
static long    hold_course       	 	= -1;		// deg * 100 dir of plane

static byte	command_index;						// current command memory location
static byte	nav_command_index;					// active nav command memory location
static byte	non_nav_command_index;				// active non-nav command memory location
static byte	nav_command_ID		= NO_COMMAND;	// active nav command ID
static byte	non_nav_command_ID	= NO_COMMAND;	// active non-nav command ID

// Airspeed
// --------
static int		airspeed;							// m/s * 100
static int		airspeed_nudge;  					// m/s * 100 : additional airspeed based on throttle stick position in top 1/2 of range
static float	airspeed_error;						// m/s * 100
static float	airspeed_fbwB;						// m/s * 100
static long 	energy_error;                       // energy state error (kinetic + potential) for altitude hold
static long		airspeed_energy_error;              // kinetic portion of energy error

// Location Errors
// ---------------
static long	bearing_error;						// deg * 100 : 0 to 36000
static long	altitude_error;						// meters * 100 we are off in altitude
static float	crosstrack_error;					// meters we are off trackline

// Battery Sensors
// ---------------
static float	battery_voltage		= LOW_VOLTAGE * 1.05;		// Battery Voltage of total battery, initialized above threshold for filter
static float 	battery_voltage1 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cell 1, initialized above threshold for filter
static float 	battery_voltage2 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2, initialized above threshold for filter
static float 	battery_voltage3 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2+3, initialized above threshold for filter
static float 	battery_voltage4 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2+3 + 4, initialized above threshold for filter

static float	current_amps;
static float	current_total;

// Airspeed Sensors
// ----------------
static float   airspeed_raw;                       // Airspeed Sensor - is a float to better handle filtering
static int     airspeed_pressure;					// airspeed as a pressure value

// Barometer Sensor variables
// --------------------------
static unsigned long 	abs_pressure;

// Altitude Sensor variables
// ----------------------
static int		sonar_alt;


static long	takeoff_altitude;

// Waypoints
// ---------
static long	wp_distance;						// meters - distance between plane and next waypoint
static long	wp_totalDistance;					// meters - distance between old and next waypoint
static long	wp_nearest ;	 // wp number ! 
static long	wp_nearest_distance;

// repeating event control
// -----------------------
static byte 		event_id; 							// what to do - see defines
static long 		event_timer; 						// when the event was asked for in ms
static uint16_t 	event_delay; 						// how long to delay the next firing of event in millis
static int 		event_repeat = 0;					// how many times to cycle : -1 (or -2) = forever, 2 = do one cycle, 4 = do two cycles
static int 		event_value; 						// per command value, such as PWM for servos
static int 		event_undo_value;					// the value used to cycle events (alternate value to event_value)

// delay command
// --------------
static long 	condition_value;						// used in condition commands (eg delay, change alt, etc.)
static long 	condition_start;
static int 	condition_rate;

// 3D Location vectors
// -------------------
static struct 	Location home;						// home location
static struct 	Location prev_WP;					// last waypoint
static struct 	Location current_loc;				// current location
static struct 	Location next_WP;					// next waypoint

static struct 	Location WP_A;
static struct 	Location WP_B;
static struct 	Location WP_C;

static struct  	Location guided_WP;					// guided mode waypoint
static struct 	Location next_nav_command;			// command preloaded
static struct 	Location next_nonnav_command;		// command preloaded
static long 	target_altitude;					// used for altitude management between waypoints
static long 	offset_altitude;					// used for altitude management between waypoints
static bool	home_is_set; 						// Flag for if we have g_gps lock and have set the home location


// IMU variables
// -------------
static float G_Dt						= 0.02;		// Integration time for the gyros (DCM algorithm)


// Performance monitoring
// ----------------------
static long 	perf_mon_timer;						// Metric based on accel gain deweighting
static int 	G_Dt_max = 0;						// Max main loop cycle time in milliseconds
static int 	gps_fix_count = 0;
static int		pmTest1 = 0;


// System Timers
// --------------
static unsigned long 	fast_loopTimer;				// Time in miliseconds of main control loop
static unsigned long 	fast_loopTimeStamp;			// Time Stamp when fast loop was complete
static uint8_t 		delta_ms_fast_loop; 		// Delta Time in miliseconds
static int 			mainLoop_count;

static unsigned long 	medium_loopTimer;			// Time in miliseconds of medium loop
static byte 			medium_loopCounter;			// Counters for branching from main control loop to slower loops
static uint8_t			delta_ms_medium_loop;

static byte 			slow_loopCounter;
static byte 			superslow_loopCounter;
static byte			counter_one_herz;

static unsigned long 	nav_loopTimer;				// used to track the elapsed time for GPS nav

static unsigned long 	dTnav;						// Delta Time in milliseconds for navigation computations
static float 			load;						// % MCU cycles used

AP_Relay relay;

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

void setup() {
	memcheck_init();
	init_ardupilot();
}

void loop()
{
	// We want this to execute at 50Hz if possible
	// -------------------------------------------
	if (millis()-fast_loopTimer > 19) {
		delta_ms_fast_loop	= millis() - fast_loopTimer;
		load                = (float)(fast_loopTimeStamp - fast_loopTimer)/delta_ms_fast_loop;
		G_Dt                = (float)delta_ms_fast_loop / 1000.f;
		fast_loopTimer      = millis();

		mainLoop_count++;

		// Execute the fast loop
		// ---------------------
		fast_loop();

		// Execute the medium loop
		// -----------------------
		medium_loop();

		counter_one_herz++;
		if(counter_one_herz == 50){
			one_second_loop();
			counter_one_herz = 0;
		}

		if (millis() - perf_mon_timer > 20000) {
			if (mainLoop_count != 0) {
				if (g.log_bitmask & MASK_LOG_PM)
					Log_Write_Performance();

				resetPerfData();
			}
		}

		fast_loopTimeStamp = millis();
	}
}

// Main loop 50Hz
static void fast_loop()
{
	// This is the fast loop - we want it to execute at 50Hz if possible
	// -----------------------------------------------------------------
	if (delta_ms_fast_loop > G_Dt_max)
		G_Dt_max = delta_ms_fast_loop;

	// Read radio
	// ----------
	read_radio();

    // try to send any deferred messages if the serial port now has
    // some space available    
    gcs_send_message(MSG_RETRY_DEFERRED);

	// check for loss of control signal failsafe condition
	// ------------------------------------
	check_short_failsafe();
	

		gcs_update();


	// uses the yaw from the DCM to give more accurate turns
	calc_bearing_error();

	// inertial navigation
	// ------------------
	#if INERTIAL_NAVIGATION == ENABLED
		// TODO: implement inertial nav function
	//	inertialNavigation();
	#endif

	// custom code/exceptions for flight modes
	// ---------------------------------------
	update_current_flight_mode();


	// write out the servo PWM values
	// ------------------------------
	set_servos();

    gcs_update();
    gcs_data_stream_send(45,1000);
}

static void medium_loop()
{
#if MOUNT == ENABLED
//	camera_mount.update_mount_position();
#endif

	// This is the start of the medium (10 Hz) loop pieces
	// -----------------------------------------
	switch(medium_loopCounter) {

		// This case deals with the GPS
		//-------------------------------
		case 0:
			medium_loopCounter++;
			if(GPS_enabled)		update_GPS();

			break;

		// This case performs some navigation computations
		//------------------------------------------------
		case 1:
			medium_loopCounter++;

                        //BUZZ TODO re-instate the test for GPS lock first ! 
			//if(g_gps->new_data){
				g_gps->new_data 	= false;
				dTnav 				= millis() - nav_loopTimer;
				nav_loopTimer 		= millis();

				// calculate the plane's desired bearing
				// -------------------------------------
				navigate();
			//}

			break;

		// command processing
		//------------------------------
		case 2:
			medium_loopCounter++;

			// Read altitude from sensors
			// ------------------
			update_alt();

			// altitude smoothing
			// ------------------
			if (control_mode != FLY_BY_WIRE_B)
				calc_altitude_error();

			// perform next command
			// --------------------
			update_commands();
			break;

		// This case deals with sending high rate telemetry
		//-------------------------------------------------
		case 3:
			medium_loopCounter++;

			if (g.log_bitmask & MASK_LOG_NTUN)
				Log_Write_Nav_Tuning();

			if (g.log_bitmask & MASK_LOG_GPS)
				Log_Write_GPS(g_gps->time, current_loc.lat, current_loc.lng, g_gps->altitude, current_loc.alt, (long) g_gps->ground_speed, g_gps->ground_course, g_gps->fix, g_gps->num_sats);

            // send all requested output streams with rates requested
            // between 5 and 45 Hz
            gcs_data_stream_send(5,45);
			break;

		// This case controls the slow loop
		//---------------------------------
		case 4:
			medium_loopCounter = 0;
			delta_ms_medium_loop	= millis() - medium_loopTimer;
			medium_loopTimer      	= millis();

			if (g.battery_monitoring != 0){
				read_battery();
			}

			slow_loop();
			break;
	}
}

static void slow_loop()
{
	// This is the slow (3 1/3 Hz) loop pieces
	//----------------------------------------
	switch (slow_loopCounter){
		case 0:
			slow_loopCounter++;
			//check_long_failsafe();
			superslow_loopCounter++;
			if(superslow_loopCounter >=200) {				//	200 = Execute every minute
				#if HIL_MODE != HIL_MODE_ATTITUDE
					//if(g.compass_enabled) {
					//	compass.save_offsets();
					//}
				#endif

				superslow_loopCounter = 0;
			}
			break;

		case 1:
			slow_loopCounter++;

			// Read 3-position switch on radio
			// -------------------------------
			read_control_switch();

			// Read Control Surfaces/Mix switches
			// ----------------------------------
			update_servo_switches();

			update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);

			break;

		case 2:
			slow_loopCounter = 0;
			update_events();

            mavlink_system.sysid = g.sysid_this_mav;		// This is just an ugly hack to keep mavlink_system.sysid sync'd with our parameter
            gcs_data_stream_send(3,5);
			break;
	}
}

static void one_second_loop()
{
	if (g.log_bitmask & MASK_LOG_CUR)
		Log_Write_Current();

	// send a heartbeat
	gcs_send_message(MSG_HEARTBEAT);
    gcs_data_stream_send(1,3);
}

static void update_GPS(void)
{
	g_gps->update();
	update_GPS_light();

	if (g_gps->new_data && g_gps->fix) {
		// for performance
		// ---------------
		gps_fix_count++;

		if(ground_start_count > 1){
			ground_start_count--;
			ground_start_avg += g_gps->ground_speed;

		} else if (ground_start_count == 1) {
			// We countdown N number of good GPS fixes
			// so that the altitude is more accurate
			// -------------------------------------
			if (current_loc.lat == 0) {
				ground_start_count = 5;

			} else {
				if(ENABLE_AIR_START == 1 && (ground_start_avg / 5) < SPEEDFILT){
					startup_ground();

					if (g.log_bitmask & MASK_LOG_CMD)
						Log_Write_Startup(TYPE_GROUNDSTART_MSG);

					init_home();
				} else if (ENABLE_AIR_START == 0) {
					init_home();
				}

				ground_start_count = 0;
			}
		}


		current_loc.lng = g_gps->longitude;    // Lon * 10**7
		current_loc.lat = g_gps->latitude;     // Lat * 10**7

	}
}

static void update_current_flight_mode(void)
{
	if(control_mode == AUTO){
		//crash_checker();

		switch(nav_command_ID){
			case MAV_CMD_NAV_TAKEOFF:
				break;

			case MAV_CMD_NAV_LAND:
				break;

			default:
				hold_course = -1;
				break;
		}
	}else{
		switch(control_mode){
			case RTL:
			case LOITER:
			case GUIDED:
				hold_course = -1;
				break;

			case FLY_BY_WIRE_A:
				break;

			case FLY_BY_WIRE_B:
				break;

			case STABILIZE:
				break;

			case CIRCLE:
				break;

			case MANUAL:

				break;

		}
	}
}



static void update_alt()
{
	#if HIL_MODE == HIL_MODE_ATTITUDE
		current_loc.alt = g_gps->altitude;
	#else

		current_loc.alt = (1 - g.altitude_mix) * g_gps->altitude;			// alt_MSL centimeters (meters * 100)
		current_loc.alt += g.altitude_mix * (read_barometer() + home.alt);
	#endif

}
