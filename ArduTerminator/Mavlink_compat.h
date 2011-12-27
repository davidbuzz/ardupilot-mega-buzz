/*
  compatibility header during transition to MAVLink 1.0
 */

#ifdef MAVLINK10

#else

static uint8_t mav_var_type(AP_Meta_class::Type_id t)
{
	return 0;
}

#define MAV_MISSION_ACCEPTED 0
#define MAV_MISSION_UNSUPPORTED       1
#define MAV_MISSION_UNSUPPORTED_FRAME 1
#define MAV_MISSION_ERROR             1
#define MAV_MISSION_INVALID_SEQUENCE  1

/*
  some functions have some extra params in MAVLink 1.0
 */

static void mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon,
						 int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy,
						 int16_t vz, uint16_t hdg)
{
	mavlink_msg_global_position_int_send(
		chan,
		lat,
		lon,
		alt,
		vx, vy, vz);
}

static void mavlink_msg_rc_channels_scaled_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t port,
						int16_t chan1_scaled, int16_t chan2_scaled, int16_t chan3_scaled,
						int16_t chan4_scaled, int16_t chan5_scaled, int16_t chan6_scaled,
						int16_t chan7_scaled, int16_t chan8_scaled, uint8_t rssi)
{
    mavlink_msg_rc_channels_scaled_send(
        chan,
	chan1_scaled,
	chan2_scaled,
	chan3_scaled,
	chan4_scaled,
	chan5_scaled,
	chan6_scaled,
	chan7_scaled,
	chan8_scaled,
	rssi);
}

static void mavlink_msg_rc_channels_raw_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t port,
					     uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw,
					     uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw,
					     uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi)
{
	mavlink_msg_rc_channels_raw_send(
		chan,
		chan1_raw,
		chan2_raw,
		chan3_raw,
		chan4_raw,
		chan5_raw,
		chan6_raw,
		chan7_raw,
		chan8_raw,
		rssi);
}


static void mavlink_msg_servo_output_raw_send(mavlink_channel_t chan, uint32_t time_usec, uint8_t port,
					      uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw,
					      uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw,
					      uint16_t servo7_raw, uint16_t servo8_raw)
{
        mavlink_msg_servo_output_raw_send(
		chan,
		servo1_raw,
		servo2_raw,
		servo3_raw,
		servo4_raw,
		servo5_raw,
		servo6_raw,
		servo7_raw,
		servo8_raw);
}

static void mavlink_msg_statustext_send(mavlink_channel_t chan, uint8_t severity, const char *text)
{
        mavlink_msg_statustext_send(chan, severity, (const int8_t*) text);
}

static void mavlink_msg_param_value_send(mavlink_channel_t chan, const char *param_id,
					 float param_value, uint8_t param_type,
					 uint16_t param_count, uint16_t param_index)
{
	mavlink_msg_param_value_send(
		chan,
		(int8_t *)param_id,
		param_value,
		param_count,
		param_index);
}
#endif
