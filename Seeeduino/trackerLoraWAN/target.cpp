#include "target.h"
#include "gps.h"
#include "log.h"

int currentTarget;
int numTargets;
Target targets[MAXTARGETS];

void initTargets(void) {
    // Dummy initialization
    numTargets=MAXTARGETS;
    targets[0].setPosition(90,0,now());  // North pole
    for (int i=1;i<numTargets;i++)
	targets[i].setPosition(37.44479,-122.17686,now());
    currentTarget = 0;  // Compass
    currentTarget = 4;  // TODO Testing
}

float Target::getDistance(void) {  // Get distance from our current position to this target in meters
    float mylat, mylong;
    gps.f_get_position(&mylat, &mylong);
    return gps.distance_between(mylat,mylong,lat,lon);
}

float Target::getHeading(void) {  // Get heading from our position to the target
    float mylat, mylong;
    gps.f_get_position(&mylat, &mylong);
    return gps.course_to(mylat,mylong,lat,lon);
}

static int parse3(unsigned char *buf) {
    int val=buf[0]<<16|buf[1]<<8|buf[2];
    val=(val<<8)>>8;
    return val;
}

static unsigned short parse2(unsigned char *buf) {
    unsigned short val=buf[0]<<8|buf[1];
    return val;
}

void Target::processMessage(int n, unsigned char *data) {   // Process message from gateway
    if (n!=8) {
	warning("Target update message with 1+%d bytes\n",n);
	return;
    }
    lat=parse3(&data[0])/10000.0;
    lon=parse3(&data[3])/10000.0;
    int age=parse2(&data[6]);
    lastfix=now()-age;
    notice("Updated target to (%.4f,%.4f) age %d\n", lat, lon, age);
}

void Target::dump() {
    printf("%.4f,%.4f %d sec;  H=%.0f, D=%.0f\n", lat, lon, now()-lastfix,getHeading(),getDistance());
}

void targetcommand(const char *cmd) {
    if (cmd[0]=='d' || cmd[0]=='D') {
	float mylat, mylong;
	gps.f_get_position(&mylat, &mylong);
	printf("My location: %.4f,%.4f\n", mylat, mylong);
	for (int i=0;i<MAXTARGETS;i++) {
	    printf("%d: ", i);
	    targets[i].dump();
	}
    } else
	warning("Expected TD\n");
}
