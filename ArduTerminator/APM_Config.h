// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If you wish to change any of the setup parameters from
// their default values, place the appropriate #define statements here.

// For example if you wanted the Port 3 baud rate to be 38400 you would add a statement like the one below (uncommented)
//#define SERIAL3_BAUD        38400

#define GPS_PROTOCOL  GPS_PROTOCOL_MTK16

#define GCS_PROTOCOL		GCS_PROTOCOL_MAVLINK

#define SERIAL0_BAUD        115200
#define SERIAL3_BAUD         57600

#define FLIGHT_MODE_1         AUTO
#define FLIGHT_MODE_2         AUTO
#define FLIGHT_MODE_3         AUTO
#define FLIGHT_MODE_4         AUTO
#define FLIGHT_MODE_5         AUTO
//#define FLIGHT_MODE_6         MANUAL
#define FLIGHT_MODE_6         TERMINATE

#define ENABLE_AIR_START    1

#define LOGGING_ENABLED 0
#define MOUNT 0

// WE REUSE THE THROTTLE FAILSAFE CODE TO HOOK IN THE "GPS BOUNDARY FAILSAFE" INSTEAD, SO ENABLE IT: 
#define THROTTLE_FAILSAFE		ENABLED  

# define CLI_ENABLED DISABLED
# define CLI_SLIDER_ENABLED DISABLED

#define LOG_ATTITUDE_FAST   DISABLED
#define LOG_ATTITUDE_MED    DISABLED
#define LOG_GPS             DISABLED
#define LOG_PM              DISABLED
#define LOG_CTUN            DISABLED
#define LOG_NTUN            DISABLED
#define LOG_MODE            DISABLED
#define LOG_RAW             DISABLED
#define LOG_CMD             DISABLED
#define LOG_CUR		    DISABLED



