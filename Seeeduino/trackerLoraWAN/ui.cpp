#include "globals.h"
#include "ui.h"
#include "imu.h"
#include "target.h"

enum  { UI_INACTIVE, UI_STARTING, UI_ACTIVE, UI_SELECTED } uimode;
static unsigned long lastChange=0;
static int selected = -1;
const int spacing = 45;  // Angle between settings

void uiupdatestate(void) {
    if (uimode==UI_STARTING && (millis()-lastChange > 5000)) {
	notice("UI: Starting->Active\n");
	uimode=UI_ACTIVE;
    }
    if (uimode==UI_SELECTED && (millis()-lastChange > 2000)) {
	notice("UI: Selected->Active\n");
	uimode=UI_ACTIVE;
    }
}

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
	    notice("UI setting %d -> %d\n", currsetting, newsetting);
	    currsetting=newsetting;
	}
    }
    return currsetting;
}

// Register a tap in the UI with the device oriented with given tilt
// Can also retrieve overall position (acc_*) from imu
void uitap() {
    float tilt=imu.getTilt();
    notice("uitap: tilt=%.0f\n",tilt);
    if (tilt>70 && tilt<110) {
	if (uimode==UI_INACTIVE) {
	    uimode=UI_STARTING;
	    lastChange=millis();
	    notice("UI: Starting\n");
	} else if (uimode==UI_ACTIVE) {
	    selected=getuisetting();
	    notice("UI: Selected %d\n",selected);
	    if (selected >= 0 && selected<=numTargets)
		currentTarget=selected;
	    else
		warning("Bad target selected: %d",selected);
	    uimode=UI_SELECTED;
	    lastChange=millis();
	}
    } else if (uimode!=UI_INACTIVE && tilt<45) {
	notice("UI: Inactive\n");
	uimode=UI_INACTIVE;
    } else {
	notice("UI: Ignored tap, tilt=%.0f",tilt);
    }
}


// Get current position to point arrow (0-360, 0=north indicator on dial)
int getuipos(void) {
    if (uimode==UI_STARTING) 
	return (millis()-lastChange)*36/500;
    else if (uimode==UI_SELECTED)
	return sin(6.28*(millis()-lastChange)/500)*(spacing/4.0)+selected*spacing;
    else {
	return getuisetting()*45;
    }
}

