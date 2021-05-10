#include <Arduino.h>
#include "ui.h"
#include "imu.h"

bool uimode=false;
static unsigned long uistart=0;

// Register a tap in the UI with the device oriented with given tilt
// Can also retrieve overall position (acc_*) from imu
void uitap(float tilt) {
    if (tilt>80 && tilt<110) {
	if (!uimode) {
	    uimode=true;
	    uistart=millis();
	    SerialUSB.println("Entered UI mode");
	}
    } else if (uimode) {
	SerialUSB.println("Exitted UI mode");
	uimode=false;
    }
}

// Get current position to point arrow (0-360, 0=up)
int getuipos(void) {
    if (millis()-uistart < 5000)
	return (millis()-uistart)*36/500;
    else {
	float rotation = atan2(acc_y,acc_x)*57.3;
	return (int)(-rotation);
    }
}

