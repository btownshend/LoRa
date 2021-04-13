
#include "LoRaWanBT.h"

void setup(void)
{
  SerialUSB.begin(115200);
  lora.init();
  delay(2000);
  SerialUSB.println("Tracker");
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

char fmtbuf[100];

void loop(void)
{
  if (millis() - lastSend > TXTIME  && buflen == 0) {
    int sendStart = millis();

    bool stat = lora.transferPacketP2PMode((unsigned char *)&pcntr, 4, 1);
    lastSend = millis();
    lora.sendCommand("AT+TEST=RXLRPKT\r\n");

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
  if (Serial1.available()) {
    bool hiteol = lora.readLine(buf, BUFSIZE, 1);
    processPacket(buf);
    SerialUSB.println("");
  }
}

int len, rssi, snr;
unsigned short data[256];

void processPacket(char *buf) {
  SerialUSB.print("Process: ");
  SerialUSB.println(buf);
  if (strncmp(buf, "+TEST: LEN:", 11)==0) {
    int ns = sscanf(buf, "+TEST: LEN:%d, RSSI:%d, SNR:%d", &len, &rssi, &snr);
    if (ns != 3)
      SerialUSB.println("Failed Len scan");
    else {
      sprintf(fmtbuf, "Len=%d, RSSI=%d, SNR=%d", len, rssi, snr);
      SerialUSB.println(fmtbuf);
    }
  } else if (strncmp(buf, "+TEST: RX \"", 11)==0) {
    char *ptr = &buf[11];
    if (ptr[len * 2] != '"')
      SerialUSB.println("RX wrong length");
    else {
      for (int i = 0; i < len; i++) {
        char tmp[3];
        tmp[0]=ptr[i*2];tmp[1]=ptr[i*2+1];tmp[2]=0;
        SerialUSB.println(i,DEC);
        sscanf(tmp, "%2hx", &data[i]);
      }
      sprintf(fmtbuf, "Got data: 0x%02x ...", data[0]);
      SerialUSB.println(fmtbuf);
    }
  } else if (strcmp(buf, "+TEST: RXLRPKT")==0) {
    ;  // Ignore
  } else {
    SerialUSB.print("Unparsed message: ");
    SerialUSB.println(buf);
  }
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
