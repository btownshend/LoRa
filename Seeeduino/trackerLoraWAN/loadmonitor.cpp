#include <Arduino.h>

#include "loadmonitor.h"
#include "globals.h"

void loadmonitorloop(void) {
    static unsigned long lasttime=0, avgstart=0;
    static unsigned int mintime=999999, maxtime=0,samples=0;
    unsigned int interval=(unsigned int)(millis()-lasttime);
    if (interval<mintime)
	mintime=interval;
    else if (interval>maxtime)
	maxtime=interval;
    samples++;
    if (samples>=100) {
	sprintf(fmtbuf,"min=%d,max=%d,avg=%ld", mintime, maxtime, (millis()-avgstart)/samples);
	samples=0;
	avgstart=millis();
	mintime=99999;
	maxtime=0;
    }
    yield();
}
