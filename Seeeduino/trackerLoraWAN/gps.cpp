#include "gps.h"

TinyGPS gps;
int gpsecho = 20;

bool getgps() {
  // Parse any available data from GPS receiver
  // Set lat, lon if found and return true, else false
  bool newData = false;
  static char gpsline[150];
  static unsigned int gpslinelen = 0;

  while (Serial2.available()) {
    char c = Serial2.read();

    if (gps.encode(c)) {
      newData = true;
    }

    if (gpsecho > 0) {
      if (c == '\r' || c == '\n') {
        if (gpslinelen > 0) {
          gpsline[gpslinelen] = 0;
          SerialUSB.print("<GPS: ");
          SerialUSB.print(gpsline);
          if (gpslinelen == sizeof(gpsline) - 1 )
            SerialUSB.print("<trunc> ");
          if (Serial2.available() > 2) {
            SerialUSB.print("<avail:");
            SerialUSB.print(Serial2.available());
            SerialUSB.print(">");
          }
          if (newData)
            SerialUSB.println("<newdata>");
          else
            SerialUSB.println("");
          gpslinelen = 0;
          gpsecho--;
          if (gpsecho == 0)
            SerialUSB.println("Disabling GPS echo until next GPS command received");
        }
      } else if (gpslinelen < sizeof(gpsline) - 1 )
        gpsline[gpslinelen++] = c;
    }
  }
  return newData;
}

