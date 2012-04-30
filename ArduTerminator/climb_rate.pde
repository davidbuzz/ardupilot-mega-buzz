// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if 0 // currently unused

struct DataPoint {
	unsigned long	x;
	long			y;
};

DataPoint		history[ALTITUDE_HISTORY_LENGTH]; // Collection of (x,y) points to regress a rate of change from
unsigned char	hindex; // Index in history for the current data point

unsigned long	xoffset;
unsigned char	n;

// Intermediate variables for regression calculation
long			xi;
long			yi;
long			xiyi;
unsigned long	xi2;

#endif

