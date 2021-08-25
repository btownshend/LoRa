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

#include "wiring_private.h" // pinPeripheral() function

#ifdef SEEEDUINOBOARD
// Add SerialExt using SERCOM2
Uart SerialExt( &sercom2, PIN_SERIALEXT_RX, PIN_SERIALEXT_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2 ) ;
void SERCOM2_Handler() {
  SerialExt.IrqHandler();
}
#endif
#ifdef PROTOBOARD
// Add SerialExt using SERCOM4 (Grove connector)
Uart SerialExt( &sercom4, PIN_SERIALEXT_RX, PIN_SERIALEXT_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2 ) ;
void SERCOM4_Handler() {
  SerialExt.IrqHandler();
}
#endif


char fmtbuf[200]; // Space to build formatted strings

void cmdexec(char *buf) {
  notice("Exec: %s\n", buf);
  if (buf[0] == 'L')
    lorawanusercommand(buf + 1);
  else if (buf[0] == 'G')
    gpsusercommand(buf + 1);
  else if (buf[0] == 'I')
    imu.command(buf + 1);
  else if (buf[0] == 'N')
    needle.command(buf + 1);
  else if (buf[0] == 'T')
    targetcommand(buf + 1);
#ifdef EXTERNALBLE
  else if (buf[0] == 'B')
    blecommand(buf + 1);
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
    warning("%s: stack decreased from %d to %d\n", module, minstack, Scheduler.stack());
    minstack = Scheduler.stack();
  }
  if (minstack < 100) {
    // Serious problem
    error("%s: stack overrun: %d remaining\n", module, minstack);
  }
  return minstack;
}

void statusLine(const char *line) {
  SerialUSB.print("** ");
  SerialUSB.println(line);
#ifdef EXTERNALBLE
  if (bleconnected) {
    SerialBLE.println(line);
  }
#endif
}

void statusReport(void) {
  // Generate a status report at regular intervals
  static unsigned long lastReport = 0;
  if (millis() - lastReport > 10000) {
#ifdef EXTERNALBLE
    if (bleconnected)
      SerialBLE.println("--------------------");
#endif
    sprintf(fmtbuf, "{\"devAddr\":\"%s\",\"frame\":[%d,%d],\"date\":\"%d/%d/%d %02d:%02d:%02d\",", devAddr, ulcntr, dlcntr, month(), day(), year(), hour(), minute(), second()); statusLine(fmtbuf);
    sprintf(fmtbuf, "\"DR\":%d,\"margin\":%d,\"LCR\":%d,\"lastLCR\":%d,\"RSSI\":%d,\"SNR\":%.1f,\"lastSNR\":%d,", currentDR, gwmargin, pendingLCR, (millis() - lastLCR) / 1000, lastRSSI, lastSNR, (millis() - lastReceived) / 1000); statusLine(fmtbuf);
    sprintf(fmtbuf, "\"target\":%d,\"tgtdist\":%.0f,\"tgtheading\":%.0f,\"tgtage\":%d,", currentTarget, targets[currentTarget].getDistance(), targets[currentTarget].getHeading(), targets[currentTarget].getAge()); statusLine(fmtbuf);
    long lat, lon;   unsigned long age;
    gps.get_position(&lat, &lon, &age);
    sprintf(fmtbuf, "\"myage\":%d,\"nsat\":%d,\"head\":%d", age / 1000, gps.satellites(),(int)imu.getHeading()); statusLine(fmtbuf);
    //	sprintf(fmtbuf,"\"acc\":[%d,%d,%d],\"heading\":%.0f}",imu.acc_x,imu.acc_y,imu.acc_z,imu.getHeading());
    //	statusLine(fmtbuf);
    lastReport = millis();
  }
}

// Override _putchar from the printf library so it sends to SerialUSB (printf_init doesn't work for some reason)
extern "C" void _putchar(char character)
{
  SerialUSB.print(character);
}



void setup(void) {
  SerialUSB.begin(115200);

  delay(2000);
  SerialUSB.println("TrackerLoraWAN");
  printf("Log Level: %d\n",logLevel);

#ifdef EXTERNALGPS
  notice("External GPS\n");
#endif
#ifdef EXTERNALLORA
  notice("External LoRa\n");
#endif
#ifdef EXTERNALBLE
  notice("External BLE\n");
  blesetup();
#endif

  imu.setup();
  needle.setup();
  gpssetup();
  batterysetup();
  lorawansetup();

  // Change SerialExt pins to SERCOM (mode C) (AFTER SerialExt.begin)
#ifdef SEEEDUINOBOARD
  pinPeripheral(PIN_SERIALEXT_RX, PIO_SERCOM);
  pinPeripheral(PIN_SERIALEXT_TX, PIO_SERCOM);
#endif

#ifdef PROTOBOARD
  pinPeripheral(PIN_SERIALEXT_RX, PIO_SERCOM_ALT);
  pinPeripheral(PIN_SERIALEXT_TX, PIO_SERCOM_ALT);
#endif


  initTargets();

  if (imu.haveimu)
    Scheduler.startLoop(imuloop, 2048);
#ifndef IMUTEST
  Scheduler.startLoop(needleloop, 2048);
  Scheduler.startLoop(gpsloop);
  Scheduler.startLoop(lorawanloop, 2048); // Needs more than 1024 bytes of stack space or sprintf causes problem
  Scheduler.startLoop(loadmonitorloop);
#ifdef EXTERNALBLE
  Scheduler.startLoop(bleloop, 2048);
#endif
#endif
}

void loop(void) {
  cmdread();
#ifndef IMUTEST
  statusReport();
#endif
  yield();
}
