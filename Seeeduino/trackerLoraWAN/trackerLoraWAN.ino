#include <TinyGPS.h>

TinyGPS gps;
short myid = 1;
char fmtbuf[100];   // Space to build formatted strings
const int updateInterval = 10;   // Update interval in seconds

// battey of Seeeduino LoRaWAN
const int pin_battery_status  = A5;
const int pin_battery_voltage = A4;

unsigned short batteryvoltage() {
  unsigned short battery;

  pinMode(pin_battery_status , OUTPUT);
  digitalWrite(pin_battery_status , LOW);
  delay(100);
  battery = (analogRead(pin_battery_voltage) * 3300 * 11) >> 10;
  pinMode(pin_battery_status , INPUT);

  //SerialUSB.print("Battery: ");
  //SerialUSB.println(battery);
  return battery;
}

unsigned char batterystatus() {
  return digitalRead(pin_battery_status);
}

void setup(void)
{
  SerialUSB.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);
  delay(2000);
  SerialUSB.println("TrackerLoraWAN");
  pinMode(pin_battery_status, INPUT);
}


bool getgps() {
  // Parse any available data from GPS receiver
  // Set lat, lon if found and return true, else false
  bool newData = false;
  while (Serial2.available()) {
    char c = Serial2.read();
    //SerialUSB.print(c);
    if (gps.encode(c))
      newData = true;
  }
  return newData;
}


void loraread() {
  // Parse all available data from LoRa module
  static char lorabuf[200];
  static int lorabuflen = 0;

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

void lorawrite(char *str) {
  // Send null terminate string to LoRa module
  Serial1.println(str);
  SerialUSB.print(">LORA: ");
  SerialUSB.println(str);
}

void loramsg(int n, unsigned char data[]) {
  // Send the msg
  strcpy(fmtbuf, "AT+MSGHEX=\"");
  char *fmtptr = fmtbuf + strlen(fmtbuf);
  for (int i = 0; i < n; i++) {
    sprintf(fmtptr, "%02X ", data[i]);
    fmtptr += 3;
  }
  fmtptr[-1] = '"';
  lorawrite(fmtbuf);
}

static int lastSend = 0;

void send() {
  long lat, lon;
  unsigned long age;
  unsigned char data[100];
  unsigned char *dptr = data;

  // magnet x (0x09,0x02)
  *dptr++ = 0x09;  *dptr++ = 0x02;
  short mx = 1234;
  *dptr++ = ((unsigned char *)&mx)[1];
  *dptr++ = ((unsigned char *)&mx)[0];
  
  // battery (not in RAK set, choose 0x0c,0x01)
  // status, then voltage
  *dptr++ = 0x0c;  *dptr++ = 0x01;
  *dptr++ = batterystatus();
  unsigned short bv = batteryvoltage();
  *dptr++ = ((unsigned char *)&bv)[1];
  *dptr++ = ((unsigned char *)&bv)[0];


  // magnet y (0x0a,0x02)
  *dptr++ = 0x0a;  *dptr++ = 0x02;
  *dptr++ = ((unsigned char *)&mx)[1];
  *dptr++ = ((unsigned char *)&mx)[0];
  
  gps.get_position(&lat, &lon, &age);  // lat, long are in units of degrees*1e6, age is in milliseconds
  if (age == gps.GPS_INVALID_AGE || age > 10000) {
    unsigned long chars;
    unsigned short sentences, failed_cs;
    gps.stats(&chars, &sentences, &failed_cs);
    sprintf(fmtbuf, "No new GPS data;  age=%ld, chars=%ld, sentences=%d, failed_cs=%d", age, chars, sentences, failed_cs);
    SerialUSB.println(fmtbuf);
  } else {
    SerialUSB.println("Got GPS data");
    long alt = gps.altitude();
    SerialUSB.print("Altitude:");
    SerialUSB.println(alt, DEC);
    if (alt == gps.GPS_INVALID_ALTITUDE)
      alt = 0xffffffff;

    *dptr++ = 0x01; *dptr++ = 0x88; // GPS indicator
    lat = lat / 100;
    lon = lon / 100;
    alt = alt * 100;
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
  int sendStart = millis();
  loramsg(dptr - data, data);
  lastSend = millis();
}

void cmdexec(char *buf) {
  SerialUSB.print("Exec: ");
  SerialUSB.println(buf);
  if (buf[0] == 'L')
    lorawrite(buf + 1);
  else if (buf[0] == 'G')
    Serial2.println(buf + 1);
  else
    SerialUSB.println("Expected (L)ora or (G)PS command");
}

void cmdread(void) {
  // Listen for commands from serial port
  static char buf[100];
  static int buflen = 0;
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
  getgps();

  if (millis() - lastSend > 1000 * updateInterval) {
    // Time to try sending
    send();
  }

  loraread();
  cmdread();
  delay(1000);
}

void processMessage(int n, unsigned char *data) {
  SerialUSB.print("processMessage(");
  SerialUSB.print(n);
  SerialUSB.println(")");
}

//void oldprocessMessage(struct packet &p) {
//  sprintf(fmtbuf, "%d,%d,%d,%d,%ld,%ld,%ld", p.m.sender, p.rssi, p.snr, p.m.framecntr, p.m.lat, p.m.lon, p.m.age);
//  SerialUSB.println(fmtbuf);
//  long lat, lon;
//  unsigned long age;
//  gps.get_position(&lat, &lon, &age);
//  if (lat != gps.GPS_INVALID_ANGLE) {
//    float bearing = gps.course_to(lat / 1e6, lon / 1e6, p.m.lat / 1e6, p.m.lon / 1e6);
//    float range = gps.distance_between(lat / 1e6, lon / 1e6, p.m.lat / 1e6, p.m.lon / 1e6);
//    sprintf(fmtbuf, "bearing=%.3f, range=%.3f", bearing, range);
//    SerialUSB.println(fmtbuf);
//  }
//}

void processLoRa(char *buf) {
  //SerialUSB.print("Process: ");
  //SerialUSB.println(buf);
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
    int n = 0;
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
  } else if (strcmp(buf, "+MSGHEX: FPENDING") == 0) {
    SerialUSB.println("Incoming message");
  } else if (strncmp(buf, "+MSGHEX: RXWIN", 14) == 0) {
    SerialUSB.println(&buf[14]);  // +MSGHEX: RXWIN1, RSSI -49, SNR 6.8
  } else if (strncmp(buf, "+MSGHEX", 7) == 0) {
    ;  // Ignore
  } else {
    SerialUSB.print("Unparsed message: ");
    SerialUSB.println(buf);
  }
}
