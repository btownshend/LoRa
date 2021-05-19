#include <Scheduler.h>
#include "gps.h"
#include "ble.h"
#include "imu.h"
#include "needle.h"
#include "globals.h"
#include "battery.h"
#include "lorawan.h"
#include "loadmonitor.h"
#include "target.h"

// Add Serial3 using SERCOM2
#include "wiring_private.h" // pinPeripheral() function

Uart Serial3( &sercom2, PIN_SERIAL3_RX, PIN_SERIAL3_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2 ) ;

void SERCOM2_Handler() {
  Serial3.IrqHandler();
}

char fmtbuf[200]; // Space to build formatted strings

void cmdexec(char *buf) {
  SerialUSB.print("Exec: ");
  SerialUSB.print(buf);
  SerialUSB.print("[");
  SerialUSB.print(strlen(buf));
  SerialUSB.println("]");
  if (buf[0] == 'L')
      lorawanusercommand(buf+1);
  else if (buf[0] == 'G')
      gpsusercommand(buf+1);
  else if (buf[0]=='I')
       imu.command(buf+1);
  else if (buf[0]=='N')
       needlecommand(buf+1);
#ifdef EXTERNALBLE
  else if (buf[0]=='B')
      blecommand(buf+1);
#endif
 else
    SerialUSB.println("Expected (L)ora, (G)PS, (I)MU, (N)eedle or (B)LE command");
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

void debug(const char *line) {
    SerialUSB.println(line);
#ifdef EXTERNALBLE
    if (bleconnected)
	SerialBLE.println(line);
#endif
}

void statusReport(void) {
    // Generate a status report at regular intervals
    static unsigned long lastReport=0;
    if (millis() - lastReport > 10000) {
	sprintf(fmtbuf,"------ %d/%d/%d %02d:%02d:%02d",month(),day(),year(),hour(),minute(),second()); debug(fmtbuf);
	sprintf(fmtbuf,"DR%d, margin=%d, LCR=%d (%d sec);  RSSI=%d, SNR=%.1f (%d sec)",currentDR, gwmargin, pendingLCR, (millis()-lastLCR)/1000,lastRSSI,lastSNR,(millis()-lastReceived)/1000);debug(fmtbuf);
	sprintf(fmtbuf,"Target %d: dist=%.0fm, heading=%.0f, age=%d", currentTarget, targets[currentTarget].getDistance(), targets[currentTarget].getHeading(),targets[currentTarget].getAge()); debug(fmtbuf);
	long lat, lon;   unsigned long age;
	gps.get_position(&lat,&lon,&age);
	sprintf(fmtbuf,"GPS: age:%d, nsat=%d",age/1000,gps.satellites()); debug(fmtbuf);
	sprintf(fmtbuf,"a=[%d,%d,%d] heading=%.0f",imu.acc_x,imu.acc_y,imu.acc_z,imu.getHeading()); debug(fmtbuf);
	lastReport=millis();
    }
}

void setup(void) {
  SerialUSB.begin(115200);

  delay(2000);
  SerialUSB.println("TrackerLoraWAN");

#ifdef EXTERNALGPS
  SerialUSB.println("External GPS");
#endif
#ifdef EXTERNALLORA
  SerialUSB.println("External LoRa");
#endif
#ifdef EXTERNALBLE
  SerialUSB.println("External BLE");
  blesetup();
#endif	
  
  imu.setup();
  needlesetup();
  gpssetup();
  batterysetup();
  lorawansetup();

    // Change PA14,PA15 to SERCOM (mode C) (AFTER Serial3.begin)
  pinPeripheral(PIN_SERIAL3_RX, PIO_SERCOM);
  pinPeripheral(PIN_SERIAL3_TX, PIO_SERCOM);
  
  initTargets();
  
  if (imu.haveimu)
      Scheduler.startLoop(imuloop,2048);
  Scheduler.startLoop(needleloop,2048);
  Scheduler.startLoop(gpsloop);
  Scheduler.startLoop(lorawanloop,2048);  // Needs more than 1024 bytes of stack space or sprintf causes problem
  Scheduler.startLoop(loadmonitorloop);
#ifdef EXTERNALBLE
  Scheduler.startLoop(bleloop,2048);
#endif
}

void loop(void) {
    cmdread();
    statusReport();
    yield(); 
}
