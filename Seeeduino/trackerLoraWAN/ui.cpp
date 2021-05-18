#include "globals.h"
#include "ui.h"
#include "imu.h"

enum  { UI_INACTIVE, UI_STARTING, UI_ACTIVE, UI_SELECTED } uimode;
static unsigned long lastChange=0;
static int selected = -1;

bool uiactive(void) {
    return uimode!=UI_INACTIVE;
}

// Get current setting position (8 setttings at 45deg intervals)
int getuisetting(void) {
	float rotation = -(int)(atan2(acc_y,acc_x)*57.3);
	while (rotation<0)
	    rotation+=360;
	// Quantize to 45 deg so it clicks
	int setting = (int((rotation+22.5)/45))%8;
	return setting;
}

// Register a tap in the UI with the device oriented with given tilt
// Can also retrieve overall position (acc_*) from imu
void uitap(float tilt) {
    if (tilt>80 && tilt<110) {
	if (uimode==UI_INACTIVE) {
	    uimode=UI_STARTING;
	    lastChange=millis();
	    SerialUSB.println("UI: Starting");
	} else if (uimode==UI_ACTIVE) {
	    selected=getuisetting();
	    SerialUSB.print("UI: Selected ");
	    SerialUSB.println(selected);
	    uimode=UI_SELECTED;
	}
    } else if (uimode!=UI_INACTIVE) {
	SerialUSB.println("UI: Inactive");
	uimode=UI_INACTIVE;
    } else {
	SerialUSB.print("UI: Ignored tap, tilt=");
	SerialUSB.println(tilt);
    }
}


// Get current position to point arrow (0-360, 0=north indicator on dial)
int getuipos(void) {
    if (uimode==UI_STARTING && (millis()-lastChange > 5000)) {
	SerialUSB.println("UI: Starting->Active");
	uimode=UI_ACTIVE;
    }
    if (uimode==UI_SELECTED && (millis()-lastChange > 1000)) {
	SerialUSB.println("UI: Selected->Active");
	uimode=UI_ACTIVE;
    }
    if (uimode==UI_STARTING) 
	return (millis()-lastChange)*36/500;
    else if (uimode==UI_SELECTED)
	return sin(6.28*(millis()-lastChange)/250)*22.5+selected*45;
    else {
	return getuisetting()*45;
    }
}

