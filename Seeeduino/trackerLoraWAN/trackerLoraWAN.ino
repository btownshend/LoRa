#include <Scheduler.h>

#include "gps.h"
#include "imu.h"
#include "stepper.h"
#include "globals.h"
#include "battery.h"
#include "lorawan.h"
#include "loadmonitor.h"

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

unsigned int stackcheck(const char *module, unsigned int minstack) {
    // Check amount of stack space remaining
    if (Scheduler.stack() < minstack) {
	SerialUSB.print(module);
	SerialUSB.print(": stack decreased from ");
	SerialUSB.print(minstack);
	SerialUSB.print(" to ");
	SerialUSB.println(Scheduler.stack());
	minstack=Scheduler.stack();
    }
    if (minstack<100) {
	// Serious problem
	SerialUSB.print(module);
	SerialUSB.print(" stack overrun: ");
	SerialUSB.print(minstack);
	SerialUSB.println("remaining");
    }
    return minstack;
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
  Scheduler.startLoop(stepperloop,2048);
  Scheduler.startLoop(gpsloop);
  Scheduler.startLoop(lorawanloop,2048);  // Needs more than 1024 bytes of stack space or sprintf causes problem
  Scheduler.startLoop(loadmonitorloop);
}

void loop(void) {
    cmdread();
    yield(); 
}
