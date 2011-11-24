// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

#else // LOGGING_ENABLED

// dummy functions
static void Log_Write_Mode(byte mode) {}
static void Log_Write_Startup(byte type) {}
static void Log_Write_Cmd(byte num, struct Location *wp) {}
static void Log_Write_Current() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_GPS(	long log_Time, long log_Lattitude, long log_Longitude, long log_gps_alt, long log_mix_alt,
                            long log_Ground_Speed, long log_Ground_Course, byte log_Fix, byte log_NumSats) {}
static void Log_Write_Performance() {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
static byte get_num_logs(void) { return 0; }
static void start_new_log(byte num_existing_logs) {}
static void Log_Write_Attitude(int log_roll, int log_pitch, uint16_t log_yaw) {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Raw() {}


#endif // LOGGING_ENABLED
