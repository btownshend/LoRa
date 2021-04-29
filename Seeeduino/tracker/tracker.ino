#include <TinyGPS.h>
#include "LoRaWanBT.h"

TinyGPS gps;
short myid = 1;
char fmtbuf[100];   // Space to build formatted strings

// Packets sent over LoRa
struct msg  {
  unsigned short sender;    // ID of sender
  unsigned short framecntr;    // Sequential frame number
  long lat; // Latitude in 10^-6 deg
  long lon; // Longitude in 10^-6 deg
  unsigned long age;  // Age of fix in milliseconds
};

struct packet {
  int len, rssi, snr;
  struct msg m;
};

void setup(void)
{
  SerialUSB.begin(115200);
  Serial2.begin(9600);
  lora.init();
  delay(2000);
  SerialUSB.println("Tracker");
  lora.initP2PMode(915, SF8, BW125, 8, 8, 20);
  delay(2000);
  SerialUSB.println("P2P mode initialized");
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

static int lastSend = 0;

void send() {
  static short framecntr = 0;
  struct msg m;


  m.sender = myid;
  m.framecntr = framecntr;
  gps.get_position(&m.lat, &m.lon, &m.age);
  if (m.age == gps.GPS_INVALID_AGE || m.age > 10000) {
    unsigned long chars;
    unsigned short sentences, failed_cs;
    gps.stats(&chars, &sentences, &failed_cs);
    sprintf(fmtbuf, "No new GPS data;  age=%ld, chars=%ld, sentences=%d, failed_cs=%d", m.age, chars, sentences, failed_cs);
    SerialUSB.println(fmtbuf);
  }
  int sendStart = millis();
  bool stat = lora.transferPacketP2PMode((unsigned char *)&m, sizeof(m), 1);
  lastSend = millis();

  lora.sendCommand((char *)"AT+TEST=RXLRPKT\r\n");  // Return to listen mode
  lora.waitForResponse((char *)"+TEST: RXLRPKT", 1);
  if (!stat)
    SerialUSB.print("Failed send");
  else {
    framecntr++;
    SerialUSB.print("Sent frame  ");
    SerialUSB.print(m.framecntr);
  }
  SerialUSB.print(" took ");
  SerialUSB.println(lastSend - sendStart, DEC);
}

void loop(void)
{
  getgps();

  if (millis() - lastSend > 5000) {
    // Time to try sending
    send();
  } 

  while (Serial1.available()) {
    // Have data from LoRa module
    const int BUFSIZE = 80;
    char buf[BUFSIZE];

    bool hiteol = lora.readLine(buf, BUFSIZE, 1);
    if (hiteol)
      processLoRa(buf);
    SerialUSB.println("");
  }
}

void processMessage(struct packet &p) {
  sprintf(fmtbuf, "%d,%d,%d,%d,%ld,%ld,%ld", p.m.sender, p.rssi, p.snr, p.m.framecntr, p.m.lat, p.m.lon, p.m.age);
  SerialUSB.println(fmtbuf);
  long lat, lon;
  unsigned long age;
  gps.get_position(&lat, &lon, &age);
  if (lat != gps.GPS_INVALID_ANGLE) {
    float bearing = gps.course_to(lat / 1e6, lon / 1e6, p.m.lat / 1e6, p.m.lon / 1e6);
    float range = gps.distance_between(lat / 1e6, lon / 1e6, p.m.lat / 1e6, p.m.lon / 1e6);
    sprintf(fmtbuf, "bearing=%.3f, range=%.3f", bearing, range);
    SerialUSB.println(fmtbuf);
  }
}

void processLoRa(char *buf) {
  static struct packet p;
  SerialUSB.print("Process: ");
  SerialUSB.println(buf);
  if (strncmp(buf, "+TEST: LEN:", 11) == 0) {
    int ns = sscanf(buf, "+TEST: LEN:%d, RSSI:%d, SNR:%d", &p.len, &p.rssi, &p.snr);
    if (ns != 3)
      SerialUSB.println("Failed Len scan");
    else {
      sprintf(fmtbuf, "Len=%d, RSSI=%d, SNR=%d", p.len, p.rssi, p.snr);
      SerialUSB.println(fmtbuf);
    }
  } else if (strncmp(buf, "+TEST: RX \"", 11) == 0) {
    char *ptr = &buf[11];
    if (ptr[p.len * 2] != '"')
      SerialUSB.println("RX wrong length");
    else if (p.len == sizeof(p.m))  {
      unsigned char *data = (unsigned char *)&p.m;
      for (int i = 0; i < p.len; i++) {
        char tmp[3];
        tmp[0] = ptr[i * 2]; tmp[1] = ptr[i * 2 + 1]; tmp[2] = 0;
        unsigned short v;
        sscanf(tmp, "%2hx", &v);
        data[i] = v;
      }
      //sprintf(fmtbuf, "Got data: 0x%02x ...", data[0]);
      //SerialUSB.println(fmtbuf);
      processMessage(p);
    }
  } else if (strcmp(buf, "+TEST: RXLRPKT") == 0) {
    ;  // Ignore
  } else {
    SerialUSB.print("Unparsed message: ");
    SerialUSB.println(buf);
  }
}
