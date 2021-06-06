#include <TimeLib.h>

class Target {
    float lat, lon;
    time_t lastfix;   // Time of last fix in seconds since epoch
 public:
    Target() {lat=0;lon=0;lastfix=0; }
    void setPosition(float _lat, float _lon, time_t _lastfix) { lat=_lat; lon=_lon; lastfix=_lastfix; }
    int getAge(void) { return (int)(now()-lastfix); } // Get age of last fix in seconds
    float getDistance(void);  // Get distance from our current position to this target in meters
    float getHeading(void);  // Get heading from our position to the target
    void processMessage(int n, unsigned char *data);   // Process message from gateway
    void dump();   // Dump value to stdout
};

#define MAXTARGETS 8


extern int currentTarget;
extern int numTargets;
extern Target targets[];

extern void initTargets(void);
extern void targetcommand(const char *cmd);
