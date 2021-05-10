// Define a stepper and the pins it will use
#include <Scheduler.h>
#include <AccelStepper.h>

#include "globals.h"
#include "stepper.h"
#include "imu.h"
#include "ui.h"

AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void gotoangle(float angle) {
    const int STEPSPERREV = 720;

    int newpos = (int)(angle * STEPSPERREV / 360);
    int curpos = stepper.currentPosition();
    int change = newpos  - curpos;
    change = (change + STEPSPERREV / 2) % STEPSPERREV - STEPSPERREV / 2;
    //sprintf(fmtbuf,"cur=%d, new=%d, change=%d",curpos, newpos, change);
    //Serial.println(fmtbuf);
    stepper.moveTo(curpos + change);
}

void adjuststepper() {
    if (uimode) {
	gotoangle(getuipos());
    } else {
	// Adjust stepper accordinly
	static float priorHeading = 0;
	float heading = getHeading();
    
	if (abs(heading-priorHeading)>=5)  {
	    priorHeading=heading;
	    gotoangle(-heading);
	}
    }
}

void steppersetup() {
    const int maxspeed=1200; // X27.168 spec says maximum speed is 600 deg/s -> 1200 step/s;  use half that
    const int maxaccel=1000;  
    stepper.setMaxSpeed(maxspeed);  
    stepper.setAcceleration(maxaccel);   // Acceleration tuned for the right "look" (4000 step/s/s will get it up to vmax after rotating 90 deg, but seems to lose steps then)
    float tmax=maxspeed/maxaccel;
    float degmax=maxaccel*tmax*tmax/2;
    sprintf(fmtbuf,"Stepper setup to reach max speed of %.0f deg/sec after %.2f sec, %.0f degrees of rotation", maxspeed/2, tmax, degmax);
    SerialUSB.println(fmtbuf);
}

void stepperloop() {
    // If we have a new value, adjust stepper
    //SerialUSB.println("stepperloop");
    adjuststepper();
    stepper.run();
    //SerialUSB.println("stepperloop end");
    // Check stack
    static int minstack=100000; minstack=stackcheck("Stepper",minstack);
    yield();
}
