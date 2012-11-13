#include <sys/time.h>
#include <signal.h>
#include <unistd.h>

struct desktop_info {
//	bool slider; // slider switch state, True means CLI mode
	struct timeval sketch_start_time;
//	bool quadcopter; // use quadcopter outputs
//	unsigned framerate;
//	float initial_height;
//	bool console_mode;
};

// the state of the desktop simulation
struct desktop_info desktop_state;


long unsigned int millis(void)
{
	struct timeval tp;
	gettimeofday(&tp,NULL);
	return 1.0e3*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
		      (desktop_state.sketch_start_time.tv_sec +
		       (desktop_state.sketch_start_time.tv_usec*1.0e-6)));
}

long unsigned int micros(void)
{
	struct timeval tp;
	gettimeofday(&tp,NULL);
	return 1.0e6*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
		      (desktop_state.sketch_start_time.tv_sec +
		       (desktop_state.sketch_start_time.tv_usec*1.0e-6)));
}

void delayMicroseconds(unsigned usec)
{
	uint32_t start = micros();
	while (micros() - start < usec) {
		usleep(usec - (micros() - start));
	}
}

void delay(long unsigned msec)
{
	delayMicroseconds(msec*1000);
}



//typedef uint8_t uint8_t;

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1

void digitalWrite(uint8_t, uint8_t) { 
	
} 

void pinMode(uint8_t, uint8_t) { 
	
}

static inline uint8_t pgm_read_uint8_t(const char * s) { return (uint8_t)*s; }


#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
//#define radians(deg) ((deg)*DEG_TO_RAD)
//#define degrees(rad) ((rad)*RAD_TO_DEG)
//#define sq(x) ((x)*(x))
