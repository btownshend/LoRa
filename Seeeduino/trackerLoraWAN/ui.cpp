#include "globals.h"
#include "ui.h"
#include "imu.h"

enum  { UI_INACTIVE, UI_STARTING, UI_ACTIVE, UI_SELECTED } uimode;
static unsigned long lastChange=0;
static int selected = -1;
const int spacing = 45;  // Angle between settings

bool uiactive(void) {
    return uimode!=UI_INACTIVE;
}

// Get current setting position (8 setttings at 45deg intervals)
int getuisetting(void) {
    static int currsetting = 0;  // Keep a persistent value
    // Can update if the device is still
    int rotation = (int)imu.getUpRotation();
    int rotdiff=(currsetting*spacing-rotation+360)%360;
    if (rotdiff>40) { // Has some hysteresis
	// Quantize to spacing deg so it clicks
	int newsetting = (int((rotation+(spacing/2.0))/spacing))%8;
	if (newsetting!=currsetting) {
	    sprintf(fmtbuf,"UI setting %d -> %d", currsetting, newsetting);
	    SerialUSB.println(fmtbuf);
	    currsetting=newsetting;
	}
    }
    return currsetting;
}

// Register a tap in the UI with the device oriented with given tilt
// Can also retrieve overall position (acc_*) from imu
void uitap(float tilt) {
    notice("uitap(%f)\n",tilt);
    if (tilt>70 && tilt<110) {
	if (uimode==UI_INACTIVE) {
	    uimode=UI_STARTING;
	    lastChange=millis();
	    SerialUSB.println("UI: Starting");
	} else if (uimode==UI_ACTIVE) {
	    selected=getuisetting();
	    SerialUSB.print("UI: Selected ");
	    SerialUSB.println(selected);
	    uimode=UI_SELECTED;
	    lastChange=millis();
	}
    } else if (uimode!=UI_INACTIVE && tilt<45) {
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
    if (uimode==UI_SELECTED && (millis()-lastChange > 2000)) {
	SerialUSB.println("UI: Selected->Active");
	uimode=UI_ACTIVE;
    }
    if (uimode==UI_STARTING) 
	return (millis()-lastChange)*36/500;
    else if (uimode==UI_SELECTED)
	return sin(6.28*(millis()-lastChange)/500)*(spacing/4.0)+selected*spacing;
    else {
	return getuisetting()*45;
    }
}

