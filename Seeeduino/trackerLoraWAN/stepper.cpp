// Define a stepper and the pins it will use
#include <Scheduler.h>
#include <AccelStepper.h>
#include <SAMDTimerInterrupt.h>

#include "globals.h"
#include "stepper.h"
#include "imu.h"
#include "ui.h"

#define USEINTERRUPTS

AccelStepper stepper(AccelStepper::FULL4WIRE,PIN_STEPPER1,PIN_STEPPER2,PIN_STEPPER3,PIN_STEPPER4);
int sensorVals[360];   // Sensor values for each angle
int maxPos;  // Current angle with maximum sensor reading
const int STEPSPERREV = 720;
const int NORTHOFFSET = -5;   // Offset in steps from sensor-based zero to position with needle pointing north,, positive values rotate needle CCWt
int spinning = 0;  // Number of rotations of needle to make

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
    if (abs(change)>(stepper.isRunning()?0:4))  // Only start it moving is the required change is significant
	stepper.moveTo(curpos + change);
}

void adjuststepper() {
    if (spinning>0) {
	return;  // No adjustments while spinning
    }
    if (!isstill())
	// Unit accelerating, no update
	return;
    
    if (uiactive()) {
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
	sensorVals[i]=-1;
#ifdef USEINTERRUPTS
    setuptimer();
#endif
}

void dumpsensor(void) {
    SerialUSB.print("sensor=[");
    for (int i=0;i<360;i++) {
	SerialUSB.print(sensorVals[i]);
	if (i==359)
	    SerialUSB.print("];");
	else
	    SerialUSB.print(",");
    }
}

void sensorcheck() {
    // Check sensor

    int position=((stepper.currentPosition()-NORTHOFFSET)/(STEPSPERREV/360))%360;
    while (position<0)
	position+=360;

    static int nfound=0;
    if (sensorVals[position]==-1) {
	nfound++;
	if (nfound%10==0) {
	    SerialUSB.print("Sensed ");
	    SerialUSB.print(nfound);
	    SerialUSB.println(" positions");
	}
    }
    sensorVals[position]=analogRead(PIN_SENSOR);
    if (spinning>0 && stepper.distanceToGo()<20)
	stepper.move(20);
    
    if (nfound==360) {
	if (spinning>0)
	    spinning--;
	unsigned long tic=millis();
	// Find maximum weighted average in WINDOWSIZE window
	int s=0,sx=0;
	int peakoffset=0;
	const int WINDOWSIZE=45;
	for (int i=0;i<WINDOWSIZE;i++) {
	    s+=sensorVals[i];
	    sx+=sensorVals[i]*i;
	}
	int maxw=0, peakpos=0, peakloc=0, xsum=0;
	for (int i=0;i<360;i++) {
	    int offset=(sensorVals[i]+sensorVals[(i+1)%360]+sensorVals[(i+WINDOWSIZE-1)%360]+sensorVals[(i+WINDOWSIZE)%360])/4; // Baseline assumed to be average of first and last point
	    int sum=s-offset*WINDOWSIZE;
	    if (sum>maxw) {
		maxw=sum;
		xsum=sx-offset*WINDOWSIZE*(i+(WINDOWSIZE-1)/2);
		peakpos=(xsum/sum)%360;
		peakloc=i;
		peakoffset=offset;
	    }
	    s+=sensorVals[(i+WINDOWSIZE)%360]-sensorVals[i];
	    sx+=(sensorVals[(i+WINDOWSIZE)%360])*(i+WINDOWSIZE)-sensorVals[i]*i;
	}
	if (peakpos>180)
	    peakpos-=360;
	dumpsensor();
	sprintf(fmtbuf,"v=[v,struct('sensor',sensor,'peakpos',%d,'offset',%d,'area',%d,'xsum',%d,'range',%d:%d)];", peakpos,peakoffset,maxw,xsum,peakloc+1, peakloc+WINDOWSIZE);
	SerialUSB.println(fmtbuf);
	stepper.setCurrentPosition(stepper.currentPosition()-peakpos*STEPSPERREV/360+NORTHOFFSET);
	// Clear for another go
	for (int i=0;i<360;i++) 
	    sensorVals[i]=-1;   
	nfound=0;
	unsigned long toc=millis();
	sprintf(fmtbuf,"Analysis took %ld msec.",toc-tic);
	SerialUSB.println(fmtbuf);
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

void steppercommand(const char *cmd) {
    if (cmd[0]=='d' || cmd[0]=='D') {
	dumpsensor();
	SerialUSB.println("");
    } else if (cmd[0]=='s' || cmd[0]=='S') {
	spinning=2;
	SerialUSB.println("Spinning");
	stepper.move(STEPSPERREV/360);
    }
}
