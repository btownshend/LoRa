#include <Scheduler.h>

#include "globals.h"
#include "lorawan.h"
#include "battery.h"
#include "imu.h"
#include "gps.h"
#include "ble.h"

#ifdef EXTERNALBLE
//#define LORABLE  // Send LoRa debug messages to BLE
#endif

const int updateInterval = 10;   // Update interval in seconds
bool msgsending = false;
int maxmsglen = 53;
int currentDR = -1;
int gwmargin = 0;   // Assume 0dB margin to start
unsigned long lorabusy = 0; // Time in msec that last lora cmd was sent -- avoid overrunning
const int LORABUSYTIME = 10;  // Assume cmds take this long to run (msec)
float lastSNR = 999;  // SNR of last received message
int lastRSSI = 999;  // RSSI of last received message
unsigned long lastReceived = 0;  // Time  of last received message (millis)
int pendingLCR = 0;   // Number of LCR messages sent since last response received
unsigned long lastLCR = 0; // Last LCR messager received (millis)
char devAddr[12]="??";
int ulcntr=0, dlcntr=0;

void lorawrite(const char *str);  // Forward declaration
void processLoRa(char *buf);

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

	static int lastfrac=-1;
	if (lastfrac!=frac) {
	    // Only if the percentage has changed, update the module
	    sprintf(fmtbuf, "AT+LW=BAT,%d", frac);
	    lorawrite(fmtbuf);
	    lastfrac=frac;
	}
    }
    return true;
}


bool setDR() {
    // Set DR in Lora module if needed based on current margin
    // Return true if changed
    const int installmargin = 10;
    const int minDR = 0;
    const int maxDR = 4;

    int tgtDR;
    
    if (pendingLCR > 1)
	tgtDR = currentDR-1;
    else
	tgtDR = ((gwmargin - installmargin) - (-20)) * 2 / 5;

    if (tgtDR < minDR)
	tgtDR = minDR;
    else if (tgtDR > maxDR)
	tgtDR = maxDR;
    if (tgtDR != currentDR) {
	notice( "pendingLCR=%d, margin=%d: changing DR from %d to %d\n", pendingLCR, gwmargin, currentDR, tgtDR);
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
    notice("Join request initiated\n");
    sprintf(fmtbuf, "AT+ID", currentDR);  // Request the device address
    lorawrite(fmtbuf);
    notice("ID request initiated\n");
}

void loraread() {
    // Parse all available data from LoRa module
    static char lorabuf[200];
    static unsigned int lorabuflen = 0;

    while (SerialLoRa.available()) {
	char c = SerialLoRa.read();
	if (c == '\r') {
	    lorabuf[lorabuflen] = 0;
	    trace("<LORA: %s\n",lorabuf);
#ifdef LORATOBLE
	    if (bleconnected) {
		SerialBLE.print("<LORA: ");
		SerialBLE.println(lorabuf);
	    }
#endif
	    processLoRa(lorabuf);
	    lorabuflen = 0;
	} else if (c != '\n' && lorabuflen < sizeof(lorabuf) - 1)
	    lorabuf[lorabuflen++] = c;
    }
}

bool islorabusy() {
    return msgsending || SerialLoRa.available() || (millis() - lorabusy < LORABUSYTIME);
}

void lorawrite(const char *str) {
    // Send null terminated string to LoRa module
    if (msgsending) {
	warning("*** lorawrite while msgsending is true; clearing\n");
	msgsending = false;
    }

    while (islorabusy()) {
	warning("*** Lora busy with prior command, delaying\n");
	loraread();
	delay(1);
    }
    SerialLoRa.println(str);
    lorabusy = millis();

    trace(">LORA: %s\n",str);
#ifdef LORATOBLE
    if (bleconnected) {
	SerialBLE.print(">LORA: ");
	SerialBLE.println(str);
    }
#endif
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
    unsigned long timeout = millis()+10000;   // Give 10s to get ack from module
    while (msgsending && millis()<timeout) {
	loraread();
	delay(1);   // At 115,200 baud, 10 chars would take ~1ms
    }
    if (msgsending) {
	error("*** Timeout while sending message\n");
	msgsending=false;
    }
}

static int lastSend = 0;

void send() {
    long lat, lon;
    unsigned long age;
    static unsigned char data[100];
    unsigned char *dptr = data;

    // Update counters
    lorawrite("AT+LW=ULDL");
    
    if ((int)sizeof(data)-1 < maxmsglen)
	maxmsglen=sizeof(data)-1;   // Avoid overflows
  
    gps.get_position(&lat, &lon, &age);  // lat, long are in units of degrees*1e6, age is in milliseconds
    if (age == gps.GPS_INVALID_AGE || age > 10000) {
	unsigned long chars;
	unsigned short sentences, failed_cs;
	gps.stats(&chars, &sentences, &failed_cs);
	notice( "No new GPS data;  age=%ld, chars=%ld, sentences=%d, failed_cs=%d\n", age, chars, sentences, failed_cs);
    } else {
	trace("Got GPS data\n");
	long alt = gps.altitude();  // Altitude in cm
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

    if (imu.haveimu) {
	if (maxmsglen > 12 + (dptr - data)) {
	    // magnet x (0x09,0x02)
	    *dptr++ = 0x09;  *dptr++ = 0x02;
	    *dptr++ = ((unsigned char *)&imu.mag_x)[1];
	    *dptr++ = ((unsigned char *)&imu.mag_x)[0];
	    // magnet y (0x0a,0x02)
	    *dptr++ = 0x0a;  *dptr++ = 0x02;
	    *dptr++ = ((unsigned char *)&imu.mag_y)[1];
	    *dptr++ = ((unsigned char *)&imu.mag_y)[0];
	    // magnet z (0x0b,0x02)
	    *dptr++ = 0x0b;  *dptr++ = 0x02;
	    *dptr++ = ((unsigned char *)&imu.mag_z)[1];
	    *dptr++ = ((unsigned char *)&imu.mag_z)[0];
	}

	if (maxmsglen > 8 + (dptr - data)) {
	    // accel
	    *dptr++ = 0x03;  *dptr++ = 0x71;
	    *dptr++ = ((unsigned char *)&imu.acc_x)[1];
	    *dptr++ = ((unsigned char *)&imu.acc_x)[0];
	    *dptr++ = ((unsigned char *)&imu.acc_y)[1];
	    *dptr++ = ((unsigned char *)&imu.acc_y)[0];
	    *dptr++ = ((unsigned char *)&imu.acc_z)[1];
	    *dptr++ = ((unsigned char *)&imu.acc_z)[0];
	}

	if (maxmsglen > 8 + (dptr - data)) {
	    // gyro
	    *dptr++ = 0x05;  *dptr++ = 0x86;
	    *dptr++ = ((unsigned char *)&imu.gyro_x)[1];
	    *dptr++ = ((unsigned char *)&imu.gyro_x)[0];
	    *dptr++ = ((unsigned char *)&imu.gyro_y)[1];
	    *dptr++ = ((unsigned char *)&imu.gyro_y)[0];
	    *dptr++ = ((unsigned char *)&imu.gyro_z)[1];
	    *dptr++ = ((unsigned char *)&imu.gyro_z)[0];
	}
    }
    int sendStart = millis();
    loramsg(dptr - data, data);
    lastSend = millis();
    trace("Send took %d msec\n", lastSend - sendStart);
}

void processMessage(int n, unsigned char *data) {
    notice("processMessage(%d,0x%02x...)\n", n, data[0]);
}

void processLoRa(char *buf) {
    // Process LoRa message in buf
    // Make sure not to try to send any LoRa from here as it may be called while waiting to send something else

    if (strncmp(buf, "+TEST: LEN:", 11) == 0) {
	int len, rssi, snr;
	int ns = sscanf(buf, "+TEST: LEN:%d, RSSI:%d, SNR:%d", &len, &rssi, &snr);
	if (ns != 3)
	    error("*** Failed Len scan\n");
	else {
	    notice("Len=%d, RSSI=%d, SNR=%d\n", len, rssi, snr);
	}
    } else if (strncmp(buf, "+MSGHEX: PORT: 1; RX: \"", 23) == 0) {
	char *ptr = &buf[23];
	static unsigned char data[100];
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
	notice("Max message length: %d\n",maxmsglen);
    } else if (strcmp(buf, "+MSGHEX: FPENDING") == 0) {
	notice("Incoming message\n");
    } else if (strncmp(buf, "+MSGHEX: RXWIN", 14) == 0) {
	int nm=sscanf(buf,"+MSGHEX: RXWIN1, RSSI %d, SNR %f",&lastRSSI,&lastSNR);
	lastReceived=millis();
	if (nm!=2)
	    error("Failed parse of %s\n",buf);
    } else if (strcmp(buf, "+MSGHEX: Please join network first") == 0) {
	warning("*** Not joined\n");
	join();
    } else if (strcmp(buf, "+MSGHEX: Done") == 0) {
	if (!msgsending)
	    error("*** MSGHEX done when msgsending was false\n");
	msgsending = false;
    } else if (strncmp(buf, "+MSGHEX: Length error", 21) == 0) {
	msgsending = false;
	maxmsglen = atoi(&buf[22]); // Probably due to DR0 fallback
	warning("*** Length error, max message length: %d\n",maxmsglen);
    } else if (strncmp(buf, "+MSGHEX: Link ", 14) == 0) { // e.g. +MSGHEX: Link 21, 1
	int gwcnt;
	int nr=sscanf(buf, "+MSGHEX: Link %d, %d",&gwmargin, &gwcnt);
	if (nr!=2)
	    error("Failed Link parse: %s\n",buf);
	else {
	    notice("Link margin: %d\n",gwmargin);
	}
	pendingLCR=0;  // Clear the pending LCR
	lastLCR = millis();
    } else if (strncmp(buf, "+MSGHEX: No free channel", 24) == 0) {
	warning("*** No free channel\n");
	msgsending = false;
    } else if (strncmp(buf, "+MSGHEX", 7) == 0) {
	;  // Ignore
    } else if (strncmp(buf, "+LW: BAT", 8) == 0) {
	;  // Ignore
    } else if (strncmp(buf, "+LW: LCR", 8) == 0) {
	;  // Ignore
    } else if (strncmp(buf, "+DR: ",5) == 0) {
	;  // Ignore
    } else if (strncmp(buf, "+ID: DevAddr, ",14) == 0) {
	strncpy(devAddr,&buf[14],sizeof(devAddr)+1);
	notice("devAddr=%s\n",devAddr);
    } else if (strncmp(buf, "+LW: ULDL, ",11) == 0) { // +LW: ULDL, 867, 265
	int nr=sscanf(buf, "+LW: ULDL, %d, %d",&ulcntr, &dlcntr);
	if (nr!=2)
	    error("Failed ULDL parse: %s\n",buf);
	else {
	    notice("UL: %d, DL:%d\n",ulcntr,dlcntr);
	}
    } else {
	warning("*** Unparsed message: %s\n",buf);
    }
}

static char usercmd[100]="";

void lorawanusercommand(const char *line) {
    // Command from serial port
    if (usercmd[0] != '\0')
	error("*** Overrun of usercmd buf\n");
    if (strlen(line) >= sizeof(usercmd)) {
	error("*** User command too long\n");
	return;
    }
    strcpy(usercmd,line);
}

void lorawansetup(void) {
    SerialLoRa.begin(115200); // Seeeduino needs to have rate set using AT+UART=BR,115200
    delay(100);
    loraread();
    setDR();
    join();
}
    
void lorawanloop(void)
{
    yield();

    loraread();

    // Other tasks to do when LoRa is not busy
    if (islorabusy())
	return;
    
    // Update module's battery level indication
    if (setmodulebatterylevel())
	return;

    if (maxmsglen < 53) {
	// Check if the maximum message length has increased
	static unsigned long lastlenmsg = 0;
	if (millis() - lastlenmsg > 5000) {
	    lorawrite("AT+LW=LEN");
	    lastlenmsg = millis();
	    return;
	}
    }

    // Check if data rate needs to be changes
    if (setDR())
	return;

    // Request line update every 60s (or 5s if we haven't heard from the prior attempts)
    if (millis() - lastLCR > (60+5*pendingLCR)*1000) {
	lorawrite("AT+LW=LCR");
	pendingLCR++;
	return;
    }

    // Any user commands (from terminal) to send?
    if (usercmd[0]!='\0') {
	lorawrite(usercmd);
	usercmd[0]=0;
	return;
    }

    if (millis() - lastSend > 1000 * updateInterval) {
	// Time to try sending
	send();
    }


    // Check stack
    static int minstack=100000; minstack=stackcheck("LoRaWAN",minstack);
    // Nothing was done, can sleep a bit
    delay(1);
}
