/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
static void
handle_process_nav_cmd()
{
	// reset navigation integrators
	// -------------------------


	reset_I();

		gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nav_command.id);
	switch(next_nav_command.id){

		default:
			break;
	}
}

static void
handle_process_condition_command()
{
	gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
	switch(next_nonnav_command.id){


		case MAV_CMD_CONDITION_DISTANCE:
			do_within_distance();
			break;

		case MAV_CMD_CONDITION_CHANGE_ALT:
			do_change_alt();
			break;


		default:
			break;
	}
}

static void handle_process_do_command()
{
	gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
	switch(next_nonnav_command.id){

 
		case MAV_CMD_DO_SET_HOME:
			do_set_home();
			break;

	}
}

static void handle_no_commands()
{
	gcs_send_text_fmt(PSTR("Returning to Home"));
	next_nav_command = home;
	next_nav_command.alt = read_alt_to_hold();
	next_nav_command.id = MAV_CMD_NAV_LOITER_UNLIM;
	nav_command_ID = MAV_CMD_NAV_LOITER_UNLIM;
	non_nav_command_ID = WAIT_COMMAND;
	handle_process_nav_cmd();
	
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

static bool verify_nav_command()	// Returns true if command complete
{
	switch(nav_command_ID) {
		case MAV_CMD_NAV_WAYPOINT:
			return verify_nav_wp();
			break;
		default:
			gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_nav: Invalid or no current Nav cmd"));
			return false;
			break;
	}
}

static bool verify_condition_command()		// Returns true if command complete
{
	switch(non_nav_command_ID) {
    case NO_COMMAND:
        break;

    default:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_conditon: Invalid or no current Condition cmd"));
        break;
	}
    return false;
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/


static void do_nav_wp()
{
	set_next_WP(&next_nav_command);
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/

static bool verify_land()
{

	return false;
}

static bool verify_nav_wp()
{
	hold_course = -1;
	update_crosstrack();
	if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
		gcs_send_text_fmt(PSTR("Reached Waypoint #%i"),nav_command_index);
		return true;
	}
	// add in a more complex case
	// Doug to do

	return false;
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/


static void do_change_alt()
{
	condition_rate		= next_nonnav_command.lat;
	condition_value 	= next_nonnav_command.alt;
	target_altitude		= current_loc.alt + (condition_rate / 10);		// Divide by ten for 10Hz update
	next_WP.alt 		= condition_value;								// For future nav calculations
	offset_altitude 	= 0;											// For future nav calculations
}

 
static void do_within_distance()
{
	condition_value  = next_nonnav_command.lat;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_change_alt()
{
	if( (condition_rate>=0 && current_loc.alt >= condition_value) || (condition_rate<=0 && current_loc.alt <= condition_value)) {
		condition_value = 0;
		return true;
	}
	target_altitude += condition_rate / 10;
	return false;
}

static bool verify_within_distance()
{
	if (wp_distance < condition_value){
		condition_value = 0;
		return true;
	}
	return false;
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/


static void do_jump()
{

}

static void do_set_home()
{
	if(next_nonnav_command.p1 == 1 && GPS_enabled) {
		init_home();
	} else {
		home.id 	= MAV_CMD_NAV_WAYPOINT;
		home.lng 	= next_nonnav_command.lng;				// Lon * 10**7
		home.lat 	= next_nonnav_command.lat;				// Lat * 10**7
		home.alt 	= max(next_nonnav_command.alt, 0);
		home_is_set = true;
	}
}


