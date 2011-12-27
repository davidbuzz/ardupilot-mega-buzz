// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
static byte failsafeCounter = 0;		// we wait a second to take over the throttle and send the plane circling


static void init_rc_in()
{
	// set rc reversing
	update_servo_switches();

	// set rc channel ranges
	g.channel_roll.set_angle(SERVO_MAX);
	g.channel_pitch.set_angle(SERVO_MAX);
	g.channel_rudder.set_angle(SERVO_MAX);
	g.channel_throttle.set_range(0, 100);

	// set rc dead zones
	g.channel_roll.set_dead_zone(0);
	g.channel_pitch.set_dead_zone(0);
	g.channel_rudder.set_dead_zone(0);
	g.channel_throttle.set_dead_zone(0);

	//set auxiliary ranges
	update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);
}

static void init_rc_out()
{
  // BUZ:   I don't know why this is here twice, I just leave it like this.  :-) 
  
	APM_RC.OutputCh(CH_1, 	g.channel_roll.radio_trim);					// Initialization of servo outputs
	APM_RC.OutputCh(CH_2, 	g.channel_pitch.radio_trim);
	APM_RC.OutputCh(CH_3, 	g.channel_throttle.radio_min);
	APM_RC.OutputCh(CH_4, 	g.channel_rudder.radio_trim);

	APM_RC.OutputCh(CH_5, 	g.rc_5.radio_trim);
	APM_RC.OutputCh(CH_6, 	g.rc_6.radio_trim);
	APM_RC.OutputCh(CH_7,   g.rc_7.radio_trim);
	APM_RC.OutputCh(CH_8,   g.rc_8.radio_trim);

	//APM_RC.Init();		// APM Radio initialization

	APM_RC.OutputCh(CH_1, 	g.channel_roll.radio_trim);					// Initialization of servo outputs
	APM_RC.OutputCh(CH_2, 	g.channel_pitch.radio_trim);
	APM_RC.OutputCh(CH_3, 	g.channel_throttle.radio_min);
	APM_RC.OutputCh(CH_4, 	g.channel_rudder.radio_trim);

	APM_RC.OutputCh(CH_5, 	g.rc_5.radio_trim);
	APM_RC.OutputCh(CH_6, 	g.rc_6.radio_trim);
	APM_RC.OutputCh(CH_7,   g.rc_7.radio_trim);
        APM_RC.OutputCh(CH_8,   g.rc_8.radio_trim);
}

static void read_radio()
{
         
        // this writes it all to g.channel_xxxx.radio_in (to RADIO_IN! )  variables! 
        g.channel_roll.set_pwm(APM_RC.InputCh(CH_ROLL));
        g.channel_pitch.set_pwm(APM_RC.InputCh(CH_PITCH));
	g.channel_throttle.set_pwm(APM_RC.InputCh(CH_3));
	g.channel_rudder.set_pwm(APM_RC.InputCh(CH_4));
	g.rc_5.set_pwm(APM_RC.InputCh(CH_5));
	g.rc_6.set_pwm(APM_RC.InputCh(CH_6));
	g.rc_7.set_pwm(APM_RC.InputCh(CH_7));
	g.rc_8.set_pwm(APM_RC.InputCh(CH_8));

}

static void control_failsafe(uint16_t pwm)
{
	if(g.throttle_fs_enabled == 0)
		return;

	// Check for failsafe condition based on loss of GCS control
	 if (rc_override_active) {
		if(millis() - rc_override_fs_timer > FAILSAFE_SHORT_TIME) {
			ch3_failsafe = true;
		} else {
			ch3_failsafe = false;
		}

	//Check for failsafe and debounce funky reads
	} else if (g.throttle_fs_enabled) {
		
	}
}

static void trim_control_surfaces()
{

}

static void trim_radio()
{

}
