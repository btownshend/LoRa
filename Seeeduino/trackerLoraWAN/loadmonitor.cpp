#include "globals.h"
#include "loadmonitor.h"

void loadmonitorloop(void) {
    static unsigned long lasttime=0, avgstart=0;
    static unsigned int mintime=999999, maxtime=0,samples=0;
    unsigned long now=micros();

    unsigned int interval=(unsigned int)(now-lasttime);
    if (interval>10000) {
	notice("Long thread loop: %d msec\n",interval);
    }
    
    if (interval<mintime)
	mintime=interval;
    else if (interval>maxtime)
	maxtime=interval;
    samples++;
    if (now-avgstart > 5000000) {
	float load = (now-avgstart-mintime*samples)*100/(now-avgstart);
	notice("Loop: min=%d,max=%d,avg=%ld usec -> %.0f%%\n", mintime, maxtime, (now-avgstart)/samples,load);
	samples=0;
	avgstart=now;
	mintime=99999;
	maxtime=0;
    }
    lasttime=micros();   // Don't count our exec time
    // lasttime=now;
    yield();
}
