#include <Scheduler.h>

#include "gps.h"
#include "imu.h"
#include "stepper.h"
#include "globals.h"
#include "battery.h"
#include "lorawan.h"

char fmtbuf[200]; // Space to build formatted strings

void cmdexec(char *buf) {
  SerialUSB.print("Exec: ");
  SerialUSB.println(buf);
  if (buf[0] == 'L')
      lorawanusercommand(buf+1);
  else if (buf[0] == 'G')
      gpsusercommand(buf+1);
 else
    SerialUSB.println("Expected (L)ora or (G)PS command");
}

void cmdread(void) {
  // Listen for commands from serial port
  static char buf[100];
  static unsigned int buflen = 0;
  while (SerialUSB.available()) {
    char c = SerialUSB.read();
    if (c == '\r' || c == '\n') {
      // Execute
      if (buflen > 0) {
        buf[buflen] = 0;
        cmdexec(buf);
        buflen = 0;
      }
    } else if (buflen < sizeof(buf) - 1)
      buf[buflen++] = c;
  }
}

void setup(void) {
  SerialUSB.begin(115200);

  delay(2000);
  SerialUSB.println("TrackerLoraWAN");

  imusetup();
  steppersetup();
  gpssetup();
  batterysetup();
  lorawansetup();
  
  if (haveimu)
    Scheduler.startLoop(imuloop);
  Scheduler.startLoop(stepperloop);
  Scheduler.startLoop(gpsloop);
  Scheduler.startLoop(lorawanloop);
}

void loop(void) {
    cmdread();
    yield(); 
}
