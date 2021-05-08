#include "gps.h"
#include "imu.h"

#define ENABLESTEPPER
// Define a stepper and the pins it will use
#ifdef ENABLESTEPPER
#include <AccelStepper.h>
AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
#endif

short myid = 1;
char fmtbuf[100];   // Space to build formatted strings
const int updateInterval = 10;   // Update interval in seconds

// battey of Seeeduino LoRaWAN
const int pin_battery_status  = A5;
const int pin_battery_voltage = A4;
bool msgsending = false;
int maxmsglen = 53;
int margin = 0;   // Assume 0dB margin to start
unsigned long lorabusy = 0; // Time in msec that last lora cmd was sent -- avoid overrunning
const int LORABUSYTIME = 10;  // Assume cmds take this long to run (msec)


#ifdef ENABLESTEPPER
void gotoangle(float angle) {
  const int STEPSPERREV = 720;

  int newpos = (int)(angle * STEPSPERREV / 360);
  int curpos = stepper.currentPosition();
  int change = newpos  - curpos;
  change = (change + STEPSPERREV / 2) % STEPSPERREV - STEPSPERREV / 2;
  //  sprintf(fmtbuf,"cur=%d, new=%d, change=%d",curpos, newpos, change);
  //  Serial.println(fmtbuf);
  stepper.moveTo(curpos + change);
}
#endif

void adjuststepper() {
  // Adjust stepper accordinly
  // roll/pitch in radian
  double roll = atan2((float)acc_y, (float)acc_z);
  double pitch = atan2(-(float)acc_x, sqrt((float)acc_y * acc_y + (float)acc_z * acc_z));

  double Xheading = mag_x * cos(pitch) + mag_y * sin(roll) * sin(pitch) + mag_z * cos(roll) * sin(pitch);
  double Yheading = mag_y * cos(roll) - mag_z * sin(pitch);

  const double declination = -6;   // Magnetic declination
  double heading = 180 + 57.3 * atan2(Yheading, Xheading) + declination;
  //double heading = 180 + 57.3 * atan2(mag_y, mag_x) + declination;

  double field = sqrt(1.0 * mag_x * mag_x + 1.0 * mag_y * mag_y + 1.0 * mag_z * mag_z);

#ifdef ENABLESTEPPER
  gotoangle(heading);
#endif

  static unsigned long lastdbg = 0;

  if (millis() - lastdbg > 1000 ) {
    sprintf(fmtbuf, "Roll: %.0f, Pitch: %.0f, Heading: %.0f, Field: %.0f\n", roll * 57.3, pitch * 57.3, heading, field);
    SerialUSB.println(fmtbuf);
    lastdbg = millis();
  }
}

unsigned short batteryvoltage() {
  // Not clear what pulling down the status pin is supposed to achieve
  pinMode(pin_battery_status , OUTPUT);
  digitalWrite(pin_battery_status , LOW);
  delay(1);  /// This may cause issues with losing serial chars -- @115,200 1ms = 12 char
  unsigned short battery = (analogRead(pin_battery_voltage) * 3300 * 11) >> 10;
  pinMode(pin_battery_status , INPUT);

  SerialUSB.print("Battery: ");
  SerialUSB.println(battery);
  return battery;
}

unsigned char batterystatus() {
  // 0-charging, 1-onbattery
  return digitalRead(pin_battery_status);
}


bool setmodulebatterylevel() {
  static unsigned long lasttime = 0;
  if (millis() - lasttime < 15000)
    return false;
  lasttime = millis();
  unsigned char bs = batterystatus(); // 0-charging, 1-onbattery

  // Set battery level in LoRa module so that it can respond correctly to DevStatusReq
  if (bs == 0)
    // On charge, no battery voltage
    lorawrite("AT+LW=BAT,0"); // 0=external power source, 255=unknown, 1-254 = x/254*100%
  else {
    unsigned short bv = batteryvoltage();

    const int minvolt = 3000; // 0%
    const int maxvolt = 5000; // 100%

    int frac = (int(bv) - minvolt) * 254 / (maxvolt - minvolt);
    if (frac < 0)
      frac = 0;
    else if (frac > 254)
      frac = 254;

    sprintf(fmtbuf, "AT+LW=BAT,%d", frac);
    SerialUSB.print("fmtbuf: ");
    SerialUSB.println(fmtbuf);
    lorawrite(fmtbuf);
  }
  return true;
}


void setup(void)
{
  SerialUSB.begin(115200);
  Serial1.begin(115200); // Seeeduino needs to have rate set using AT+UART=BR,115200
  Serial2.begin(9600);

#ifdef ENABLESTEPPER
  stepper.setMaxSpeed(3600);
  stepper.setAcceleration(12000 / 100);
#endif

  delay(2000);
  SerialUSB.println("TrackerLoraWAN");
  //  int wbuf = SerialUSB.availableForWrite();
  //  int wbuf1 = Serial1.availableForWrite();
  //  int wbuf2 = Serial2.availableForWrite();
  //  SerialUSB.println("Write buffers: ");
  //  SerialUSB.print(wbuf); SerialUSB.print(", ");
  //  SerialUSB.print(wbuf1); SerialUSB.print(", ");
  //  SerialUSB.println(wbuf2);

  pinMode(pin_battery_status, INPUT);
  setDR();
  join();
}

bool setDR() {
  // Set DR in Lora module if needed based on current margin
  // Return true if changed
  const int installmargin = 10;
  const int minDR = 0;
  const int maxDR = 4;
  static int currentDR = -1;

  int tgtDR = ((margin - installmargin) - (-20)) * 2 / 5;
  if (tgtDR < minDR)
    tgtDR = minDR;
  else if (tgtDR > maxDR)
    tgtDR = maxDR;
  if (tgtDR != currentDR) {
    sprintf(fmtbuf, "margin=%d: changing DR from %d to %d", margin, currentDR, tgtDR);
    SerialUSB.println(fmtbuf);
    currentDR = tgtDR;
    sprintf(fmtbuf, "AT+DR=DR%d", currentDR);
    lorawrite(fmtbuf);
    return true;
  } else
    return false;
}

void join() {
  // while (!lora.setOTAAJoin(JOIN));
  lorawrite("AT+JOIN");
  // TODO: Should verify response
  SerialUSB.println("Join request initiated");
}

void loraread() {
  // Parse all available data from LoRa module
  static char lorabuf[200];
  static unsigned int lorabuflen = 0;

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\r') {
      lorabuf[lorabuflen] = 0;
      SerialUSB.print("<LORA: ");
      SerialUSB.println(lorabuf);
      processLoRa(lorabuf);
      lorabuflen = 0;
    } else if (c != '\n' && lorabuflen < sizeof(lorabuf) - 1)
      lorabuf[lorabuflen++] = c;
  }
}

bool islorabusy() {
  return msgsending || Serial1.available() || (millis() - lorabusy < LORABUSYTIME);
}

void lorawrite(const char *str) {
  // Send null terminated string to LoRa module
  if (msgsending) {
    SerialUSB.println("*** lorawrite while msgsending is true; clearing");
    msgsending = false;
  }

  while (islorabusy()) {
    SerialUSB.println("*** Lora busy with prior command, delaying");
    loraread();
    delay(1);
  }
  Serial1.println(str);
  lorabusy = millis();

  SerialUSB.print(">LORA: ");
  SerialUSB.println(str);
}

void loramsg(int n, unsigned char data[]) {
  // Send the msg

  if (n == 0)
    lorawrite("AT+MSGHEX");  // 0-length
  else {
    strcpy(fmtbuf, "AT+MSGHEX=\"");
    char *fmtptr = fmtbuf + strlen(fmtbuf);
    for (int i = 0; i < n; i++) {
      sprintf(fmtptr, "%02X ", data[i]);
      fmtptr += 3;
    }
    fmtptr[-1] = '"';
    lorawrite(fmtbuf);
  }
  msgsending = true;
}

static int lastSend = 0;

void send() {
  long lat, lon;
  unsigned long age;
  unsigned char data[100];
  unsigned char *dptr = data;


  gps.get_position(&lat, &lon, &age);  // lat, long are in units of degrees*1e6, age is in milliseconds
  if (age == gps.GPS_INVALID_AGE || age > 10000) {
    unsigned long chars;
    unsigned short sentences, failed_cs;
    gps.stats(&chars, &sentences, &failed_cs);
    sprintf(fmtbuf, "No new GPS data;  age=%ld, chars=%ld, sentences=%d, failed_cs=%d", age, chars, sentences, failed_cs);
    SerialUSB.println(fmtbuf);
  } else {
    SerialUSB.println("Got GPS data");
    long alt = gps.altitude();  // Altitude in cm
    SerialUSB.print("Altitude:");
    SerialUSB.println(alt, DEC);
    if (alt == gps.GPS_INVALID_ALTITUDE)
      alt = 0xffffffff;

    if (maxmsglen >= 11) {
      *dptr++ = 0x01; *dptr++ = 0x88; // GPS indicator
      lat = lat / 100;
      lon = lon / 100;
      *dptr++ = ((unsigned char *)&lat)[2];  // 3-byte encoding of lat,long * 1e4
      *dptr++ = ((unsigned char *)&lat)[1];
      *dptr++ = ((unsigned char *)&lat)[0];
      *dptr++ = ((unsigned char *)&lon)[2];
      *dptr++ = ((unsigned char *)&lon)[1];
      *dptr++ = ((unsigned char *)&lon)[0];
      *dptr++ = ((unsigned char *)&alt)[2];
      *dptr++ = ((unsigned char *)&alt)[1];
      *dptr++ = ((unsigned char *)&alt)[0];
    }
  }

  if (maxmsglen > 6 + (dptr - data)) {
    // Get hdop and num sats
    unsigned long hdop = gps.hdop();
    unsigned short numsat = gps.satellites();
    if (numsat != gps.GPS_INVALID_SATELLITES) {
      *dptr++ = 0x01; *dptr++ = 0x89; // GPS HDOP/sats
      *dptr++ = ((unsigned char *)&hdop)[2];
      *dptr++ = ((unsigned char *)&hdop)[1];
      *dptr++ = ((unsigned char *)&hdop)[0];
      *dptr++ = ((unsigned char *)&numsat)[0];
    }
  }

  if (maxmsglen > 4 + (dptr - data)) {
    // battery (not in RAK set, choose 0x0c,0x01)
    // status, then voltage
    unsigned char bs = batterystatus(); // 0-charging, 1-onbattery
    unsigned short bv = batteryvoltage();
    *dptr++ = 0x0c;  *dptr++ = 0x01;
    *dptr++ = bs;

    *dptr++ = ((unsigned char *)&bv)[1];
    *dptr++ = ((unsigned char *)&bv)[0];
  }

  if (update9DOF()) {
    if (maxmsglen > 12 + (dptr - data)) {
      // magnet x (0x09,0x02)
      *dptr++ = 0x09;  *dptr++ = 0x02;
      *dptr++ = ((unsigned char *)&mag_x)[1];
      *dptr++ = ((unsigned char *)&mag_x)[0];
      // magnet y (0x0a,0x02)
      *dptr++ = 0x0a;  *dptr++ = 0x02;
      *dptr++ = ((unsigned char *)&mag_y)[1];
      *dptr++ = ((unsigned char *)&mag_y)[0];
      // magnet z (0x0b,0x02)
      *dptr++ = 0x0b;  *dptr++ = 0x02;
      *dptr++ = ((unsigned char *)&mag_z)[1];
      *dptr++ = ((unsigned char *)&mag_z)[0];
    }

    if (maxmsglen > 8 + (dptr - data)) {
      // accel
      *dptr++ = 0x03;  *dptr++ = 0x71;
      *dptr++ = ((unsigned char *)&acc_x)[1];
      *dptr++ = ((unsigned char *)&acc_x)[0];
      *dptr++ = ((unsigned char *)&acc_y)[1];
      *dptr++ = ((unsigned char *)&acc_y)[0];
      *dptr++ = ((unsigned char *)&acc_z)[1];
      *dptr++ = ((unsigned char *)&acc_z)[0];
    }

    if (maxmsglen > 8 + (dptr - data)) {
      // gyro
      *dptr++ = 0x05;  *dptr++ = 0x86;
      *dptr++ = ((unsigned char *)&gyro_x)[1];
      *dptr++ = ((unsigned char *)&gyro_x)[0];
      *dptr++ = ((unsigned char *)&gyro_y)[1];
      *dptr++ = ((unsigned char *)&gyro_y)[0];
      *dptr++ = ((unsigned char *)&gyro_z)[1];
      *dptr++ = ((unsigned char *)&gyro_z)[0];
    }
  }
  int sendStart = millis();
  loramsg(dptr - data, data);
  lastSend = millis();
  SerialUSB.print("Send took "); Serial.print(lastSend - sendStart); Serial.println(" msec");
}

void cmdexec(char *buf) {
  SerialUSB.print("Exec: ");
  SerialUSB.println(buf);
  if (buf[0] == 'L')
    lorawrite(buf + 1);
  else if (buf[0] == 'G') {
    gpsecho = 20;  // Enable echo of GPS messages
    if (buf[1])
      Serial2.println(buf + 1);
  } else
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

void loop(void)
{
  if (update9DOF()) {
    // If we have a new value, adjust stepper
    adjuststepper();
  }
#ifdef ENABLESTEPPER
  stepper.run();
#endif


  getgps();

  if (millis() - lastSend > 1000 * updateInterval) {
    // Time to try sending
    send();
  }

  loraread();
  cmdread();

  // Other tasks to do when LoRa is not busy
  if (islorabusy())
    return;
  if (setmodulebatterylevel())
    return;
  if (maxmsglen < 53) {
    // Check if the maximum message length has increase
    static unsigned long lastlenmsg = 0;
    if (millis() - lastlenmsg > 5000) {
      lorawrite("AT+LW=LEN");
      lastlenmsg = millis();
      return;
    }
  }
  if (setDR())
    return;

  static unsigned long lastLCR = 0;
  if (millis() - lastLCR > 60000) {
    lastLCR = millis();
    lorawrite("AT+LW=LCR");
    margin -= 1; // In case we don't get a reply, slowly decrease our margin
    return;
  }
}

void processMessage(int n, unsigned char *data) {
  sprintf(fmtbuf, "processMessage(%d,0x%02x...)", n, data[0]);
  SerialUSB.println(fmtbuf);
}

void processLoRa(char *buf) {
  // Process LoRa message in buf
  // Make sure not to try to send any LoRa from here as it may be called while waiting to send something else

  if (strncmp(buf, "+TEST: LEN:", 11) == 0) {
    int len, rssi, snr;
    int ns = sscanf(buf, "+TEST: LEN:%d, RSSI:%d, SNR:%d", &len, &rssi, &snr);
    if (ns != 3)
      SerialUSB.println("Failed Len scan");
    else {
      sprintf(fmtbuf, "Len=%d, RSSI=%d, SNR=%d", len, rssi, snr);
      SerialUSB.println(fmtbuf);
    }
  } else if (strncmp(buf, "+MSGHEX: PORT: 1; RX: \"", 23) == 0) {
    char *ptr = &buf[23];
    SerialUSB.println(*ptr, DEC);
    unsigned char data[100];
    unsigned int n = 0;
    for (; n < sizeof(data) && *ptr != '"' && *ptr && ptr[1]; n++, ptr += 2) {
      char tmp[3];
      tmp[0] = ptr[0]; tmp[1] = ptr[1]; tmp[2] = 0;
      unsigned short v;
      sscanf(tmp, "%2hx", &v);
      data[n] = v;
    }
    processMessage(n, data);
  } else if (strcmp(buf, "+TEST: RXLRPKT") == 0) {
    ;  // Ignore
  } else if (strncmp(buf, "+LOG", 4) == 0) {
    ;  // Ignore
  } else if (strncmp(buf, "+LW: LEN", 8) == 0) {
    maxmsglen = atoi(&buf[9]);
    SerialUSB.print("Max message length: ");
    SerialUSB.println(maxmsglen);
  } else if (strcmp(buf, "+MSGHEX: FPENDING") == 0) {
    SerialUSB.println("Incoming message");
  } else if (strncmp(buf, "+MSGHEX: RXWIN", 14) == 0) {
    SerialUSB.println(&buf[14]);  // +MSGHEX: RXWIN1, RSSI -49, SNR 6.8
  } else if (strcmp(buf, "+MSGHEX: Please join network first") == 0) {
    SerialUSB.println("Not joined");
    join();
  } else if (strcmp(buf, "+MSGHEX: Done") == 0) {
    if (!msgsending)
      SerialUSB.println("MSGHEX done when msgsending was false");
    msgsending = false;
  } else if (strncmp(buf, "+MSGHEX: Length error", 21) == 0) {
    SerialUSB.print("Length error");
    msgsending = false;
    maxmsglen = atoi(&buf[22]); // Probably due to DR0 fallback
    SerialUSB.print("Max message length: ");
    SerialUSB.println(maxmsglen);
  } else if (strncmp(buf, "+MSGHEX: Link ", 14) == 0) { // e.g. +MSGHEX: Link 21, 1
    margin = atoi(&buf[14]);
    SerialUSB.print("Link margin: ");
    SerialUSB.println(margin);
  } else if (strncmp(buf, "+MSGHEX: No free channel", 24) == 0) {
    SerialUSB.println("No free channel");
    msgsending = false;
  } else if (strncmp(buf, "+MSGHEX", 7) == 0) {
    ;  // Ignore
  } else {
    SerialUSB.print("Unparsed message: ");
    SerialUSB.println(buf);
  }
}
