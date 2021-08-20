#include "globals.h"
#ifdef EXTERNALBLE
#include "ble.h"

bool bleconnected = true;
bool inCommandMode = false;

void bleprocess(const char *buf, int buflen) {
    const char *scan[2]={"+++\r\n0\r\nOK\r\n","1\r\nOK\r\n"};
    static int scansel=0;
    static int scanstate=0;   // Where in scanning for escape we are
    for (int i=0;i<buflen;i++) {
    }
    
    SerialUSB.print("<BLE: ");
    for (int i=0;i<buflen;i++) {
	// Check for mode change indication
	if (scanstate==0) {
	    if (scan[0][0]==buf[i])
		scansel=0;
	    else if (scan[1][0]==buf[i])
		scansel=1;
	}
	if (scan[scansel][scanstate]==buf[i]) {
	    scanstate++;
	    if (scan[scansel][scanstate]==0) {
		SerialUSB.print("Mode changed to ");
		SerialUSB.println(scansel);
		inCommandMode=scansel;
		scanstate=0;
	    }
	} else
	    scanstate=0;   // Back to beginning

	if ((buf[i]>=' ' && buf[i] <0x7f) || buf[i]=='\r' || buf[i]=='\n')
	    SerialUSB.print(buf[i]);
	else {
	    SerialUSB.print("\\");
	    SerialUSB.print(((int)buf[i])/16,HEX);
	    SerialUSB.print(((int)buf[i])%16,HEX);
	}
    }
    SerialUSB.println("");
    if (strncmp(buf,"OK+LOST",7)==0) {
	SerialUSB.println("BLE disconnected");
	bleconnected=false;
    } else if (strncmp(buf,"OK+CONN",7)==0) {
	SerialUSB.println("BLE connected");
	bleconnected=true;
    }
}

void bleread() {
    // Parse all available data from Ble module
    static char blebuf[200];
    static unsigned int blebuflen = 0;
    static unsigned long lastchar = 0;
    
    if (SerialBLE.available() > OVERRUNSIZE)
	warning("Possible SerialBLE overrun, available=%d\n",SerialBLE.available());
    
    while (SerialBLE.available() && blebuflen < sizeof(blebuf)) {
	char c = SerialBLE.read();
	blebuf[blebuflen++] = c;
	lastchar=millis();
    }

    if ((blebuflen==sizeof(blebuf)) || (blebuflen>0 && (millis()-lastchar)>10)) {
	bleprocess(blebuf,blebuflen);
	blebuflen = 0;
    }
}

void blesetup() {
    SerialBLE.begin(9600);
}

void bleloop() {
    bleread();
    // Check stack
    static int minstack=100000; minstack=stackcheck("BLE",minstack);

    yield();
}

bool changeMode(bool cmdmode) {
    if (inCommandMode == cmdmode)
	return true;
    SerialBLE.println("+++");
    unsigned long timeout=millis()+1000; 
    while (inCommandMode != cmdmode && millis()<timeout) {
	yield();
    }
    if (inCommandMode != cmdmode) {
	SerialUSB.print("BLE failed to switch to command mode: ");
	SerialUSB.println(cmdmode);
	return false;
    }
    return true;
}

void blecommand(const char *cmd) {
    changeMode(true);
    SerialBLE.println(cmd);
    changeMode(false);
}

#endif // EXTERNALBLE
       
