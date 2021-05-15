#include "globals.h"
#ifdef EXTERNALBLE
#include "ble.h"

bool bleconnected = false;

void bleprocess(const char *buf, int buflen) {
    SerialUSB.print("<BLE: ");
    for (int i=0;i<buflen;i++) {
	if (buf[i]>=' ' && buf[i] <0x7f)
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

void blecommand(const char *cmd) {
    SerialBLE.print(cmd);
}

#endif // EXTERNALBLE
       
