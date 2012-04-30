// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

void ReadSCP1000(void) {}

static void init_barometer(void)
{

}

static long read_barometer(void)
{
return 0;
}

// in M/S * 100
static void read_airspeed(void)
{
	calc_airspeed_errors();
}

static void zero_airspeed(void)
{

}

#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void read_battery(void)
{

}

