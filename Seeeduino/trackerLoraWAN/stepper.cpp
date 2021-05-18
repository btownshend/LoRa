// Define a stepper and the pins it will use
#include <Scheduler.h>
#include <AccelStepper.h>
#include <SAMDTimerInterrupt.h>

#include "globals.h"
#include "stepper.h"
#include "imu.h"
#include "ui.h"

#define USEINTERRUPTS

AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
int sensorVals[360];   // Sensor values for each angle
int maxPos;  // Current angle with maximum sensor reading
const int STEPSPERREV = 720;

#ifdef USEINTERRUPTS
SAMDTimer ITimer0(TIMER_TC3);

void TimerHandler0(void)
{
  // Doing something here inside ISR
    stepper.run();
}

#define TIMER0_INTERVAL_US     100
void setuptimer()
{
  // Interval in microsecs
  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_US, TimerHandler0))
    Serial.println("Starting  ITimer0 OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer0. Select another freq. or timer");
}
#endif

void gotoangle(float angle) {
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
    const int maxspeed=1200; // X27.168 spec says maximum speed is 600 deg/s -> 1200 step/s
    const int maxaccel=1200;    // maxspeed/maxaccel gives time to reach full speed
    stepper.setMaxSpeed(maxspeed);  
    stepper.setAcceleration(maxaccel);   // Acceleration tuned for the right "look" (4000 step/s/s will get it up to vmax after rotating 90 deg, but seems to lose steps then)
    float tmax=maxspeed*1.0/maxaccel;
    float degmax=maxaccel*tmax*tmax/2/2;
    sprintf(fmtbuf,"Stepper setup to reach max speed of %d deg/sec after %.2f sec, %.0f degrees of rotation", maxspeed/2, tmax, degmax);
    SerialUSB.println(fmtbuf);
    for (int i=0;i<360;i++)
	sensorVals[i]=0;
#ifdef USEINTERRUPTS
    setuptimer();
#endif
}

void sensorcheck() {
    // Check sensor
    int sensor=analogRead(SENSORPIN);
    int position=(stepper.currentPosition()/(STEPSPERREV/360))%360;
    while (position<0)
	position+=360;
    bool removedMax = false;  // Need to check full sensorVals array for new max if true
    int oldMax = sensorVals[maxPos];
    if (position==maxPos && sensor<oldMax)
	// Removed prior maximum
	removedMax=true;
    else if (sensor > oldMax) {
	if (abs(maxPos-position)>0) {
	    sprintf(fmtbuf,"Sensor hit new max: changed from %d@%d to %d@%d",sensorVals[maxPos],maxPos,sensor,position);
	    SerialUSB.println(fmtbuf);
	}
	maxPos=position;  // New max position
    }
    // Updates sensorVals
    sensorVals[position]=sensor;
    if (removedMax) {
	// Removed old maximum, check all angles for new max
	int newMax=maxPos;
	for (int i=0;i<360;i++)
	    if (sensorVals[i] > sensorVals[maxPos])
		newMax=i;
	if (newMax!=maxPos) {
	    if (abs(maxPos-newMax)>0) {
		sprintf(fmtbuf,"Sensor lost old max: %d@%d changed to %d and new max is %d@%d",oldMax,position,sensor,sensorVals[newMax],newMax);
		SerialUSB.println(fmtbuf);
	    }
	    maxPos=newMax;
	}
    }
}

void stepperloop() {
    // If we have a new value, adjust stepper
    //SerialUSB.println("stepperloop");
    adjuststepper();
#ifndef USEINTERRUPTS
    stepper.run();
#endif
    sensorcheck();
    
    //SerialUSB.println("stepperloop end");
    // Check stack
    static int minstack=100000; minstack=stackcheck("Stepper",minstack);
    yield();
}
