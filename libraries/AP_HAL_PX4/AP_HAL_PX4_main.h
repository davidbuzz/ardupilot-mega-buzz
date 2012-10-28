#include <unistd.h>

// this is included at the end of every sketch
extern "C" __EXPORT int SKETCH_MAIN(int argc, char *argv[]);
int SKETCH_MAIN(int argc, char *argv[]) 
{
    hal.init(NULL);
    setup();
    for(;;) {
	    loop();
	    usleep(200);
    }
    return OK;
}
