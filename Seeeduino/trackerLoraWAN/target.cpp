#include "target.h"
#include "gps.h"

int currentTarget = 0;
Target targets[MAXTARGETS];

void initTargets(void) {
    // Dummy initialization
    targets[0].setPosition(37.44479,-122.17686,now());
    currentTarget = 0;
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
