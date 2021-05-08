#include <TinyGPS.h>

extern TinyGPS gps;
extern int gpsecho;

extern void gpsusercommand(const char *buf);
extern void gpssetup();
extern void gpsloop();
