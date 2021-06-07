#include "globals.h"
#include "ui.h"
#include "imu.h"
#include "target.h"

enum  { UI_INACTIVE, UI_STATUS, UI_SETSTARTFEEDBACK, UI_SETTING, UI_SELECTFEEDBACK } uimode;

static unsigned long lastChange=0;
static int selected = -1;
const int spacing = 45;  // Angle between settings

void uiupdatestate(void) {
    if (uimode==UI_SETSTARTFEEDBACK && (millis()-lastChange > 5000)) {
	notice("UI: Start feedback done\n");
	uimode=UI_SETTING;
    }
    if (uimode==UI_SELECTFEEDBACK && (millis()-lastChange > 2000)) {
	notice("UI: Select feedback done\n");
	uimode=UI_SETTING;
    }
    float tilt=imu.getTilt();
    if (uimode==UI_INACTIVE && tilt>55) {
	notice("UI: Starting status mode (tilt=%.0f)\n",tilt);
	uimode=UI_STATUS;
    } else if (uimode!=UI_INACTIVE && tilt<35) {
	notice("UI: Ending UI mode (tilt=%.0f)\n",tilt);
	uimode=UI_INACTIVE;
    }
}

bool uiactive(void) {
    uiupdatestate();
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
    uiupdatestate();
    float tilt=imu.getTilt();
    notice("uitap: tilt=%.0f\n",tilt);
    if (uimode==UI_STATUS) {
	uimode=UI_SETSTARTFEEDBACK;
	lastChange=millis();
	notice("UI: Set feedback starting\n");
    } else if (uimode==UI_SETTING) {
	selected=getuisetting();
	notice("UI: Selected %d\n",selected);
	if (selected >= 0 && selected<=numTargets)
	    currentTarget=selected;
	else
	    warning("Bad target selected: %d",selected);
	uimode=UI_SELECTFEEDBACK;
	lastChange=millis();
    } else {
	notice("UI: Ignored tap, mode=%d\n", uimode);
    }
}


// Get current position to point arrow (0-360, 0=north indicator on dial)
int getuipos(void) {
    uiupdatestate();
    if (uimode==UI_STATUS) {
	// Status feedback depending on orientation
	int rotation=(int)imu.getUpRotation();
	int submode = ((rotation+45)/90) % 4;   // 4 modes
	int val=0;
	if (submode==0)
	    // Target number
	    val=currentTarget*45;
	else if (submode==1)    {
	    // Target distance
	    float d=targets[currentTarget].getDistance();
	    val=(d*270/1000);   // 270deg rotation is 1km
	    if (val>270)
		val=270;  // Limit
	    notice("Target dist %.0f (uipos=%d)\n", d, val);
	} else if (submode==2) {
	    // Target age
	    int age=targets[currentTarget].getAge();
	    val=(age*270/45);  // Clock hand is minutes
	    if (val>270)
		val=270;  // Limit
	    notice("Target age %d (uipos=%d)\n", age, val);
	} else if (submode==3) {
	    val=0; // Testing
	}
	return (val+90*submode)%360;
    } else if (uimode==UI_SETSTARTFEEDBACK) 
	return (millis()-lastChange)*36/500;
    else if (uimode==UI_SELECTFEEDBACK)
	return sin(6.28*(millis()-lastChange)/500)*(spacing/4.0)+selected*spacing;
    else {
	return getuisetting()*45;
    }
}

