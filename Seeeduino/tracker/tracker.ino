
#include "LoRaWanBT.h"

void setup(void)
{
  SerialUSB.begin(115200);
  lora.init();
  lora.initP2PMode(915, SF8, BW125, 8, 8, 20);
  delay(2000);
  SerialUSB.println("P2P mode initialized");
}

int pcntr = 0;
int lastSend = 0;
const int BUFSIZE = 80;
char buf[BUFSIZE];
int buflen = 0;
const int TXTIME = 10000;

void loop(void)
{
  if (millis() - lastSend > TXTIME  && buflen == 0) {
    int sendStart = millis();

    bool stat = lora.transferPacketP2PMode((unsigned char *)&pcntr, 4, 1);
    lastSend = millis();
    Serial1.println("AT+TEST=RXLRPKT");
    SerialUSB.println("AT+TEST=RXLRPKT");

    if (!stat)
      SerialUSB.print("Failed send");
    else {
      pcntr++;
      SerialUSB.print("Send pcntr ");
      SerialUSB.print(pcntr);
    }
    SerialUSB.print(" took ");
    SerialUSB.println(lastSend - sendStart, DEC);
  } else if (millis() - lastSend > TXTIME + 1000) {
    SerialUSB.print("Waiting to send but buffer contains ");
    SerialUSB.print(buflen, DEC);
    SerialUSB.println(" bytes.");
    buflen = 0;
  }
  while (Serial1.available()) {
    buf[buflen] = Serial1.read();

    if (buf[buflen] == '\n' || buf[buflen] == '\r') {
      if (buflen > 0)
        processPacket(buf, buflen);
    } else {
      buflen = buflen + 1;
      if (buflen > BUFSIZE)  {
        SerialUSB.println("Overflow");
        buflen = 0;
      }
    }
  }
}

void processPacket(char *buf, int len) {
  SerialUSB.print("Got line: ");
  buf[len] = 0;
  SerialUSB.println(buf);
  buflen = 0;
}

void extra() {
  short length = 0;
  short rssi = 0;
  unsigned char buffer[128];
  
  memset(buffer, 0, 128);
  length = lora.receivePacketP2PMode(buffer, 128,  &rssi, 5);

  if (length == 4) {
    SerialUSB.print("RSSI is: ");
    SerialUSB.print(rssi);
    SerialUSB.print("cntr: ");
    SerialUSB.println(*(int *)buffer);
    delay(1000);
  } else if (length) {
    SerialUSB.print("Length is: ");
    SerialUSB.println(length);
    SerialUSB.print("RSSI is: ");
    SerialUSB.println(rssi);
    SerialUSB.print("Data is: ");
    for (unsigned char i = 0; i < length; i ++)
    {
      SerialUSB.print("0x");
      SerialUSB.print(buffer[i], HEX);
      SerialUSB.print(" ");
    }
    SerialUSB.println();
  }
}
