#include <NMEAGPS.h>

NMEAGPS gps;
gps_fix fix;

void setup()
{
  SerialUSB.begin(115200);
  Serial2.begin(9600);
  delay(2000);
  SerialUSB.println("gpstrack");
}

char fmtbuf[200];

void loop()
{
  while (gps.available( Serial2 )) {
    fix = gps.read();
    if (fix.valid.location) {
      sprintf(fmtbuf, "status: %d, lat:%f, long:%f", fix.status, fix.latitude(), fix.longitude());
      SerialUSB.println(fmtbuf);
    } else {
      sprintf(fmtbuf, "status: %d", fix.status);
      SerialUSB.println(fmtbuf);
    }
  }
}
