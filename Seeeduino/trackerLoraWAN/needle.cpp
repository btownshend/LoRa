// Define a stepper and the pins it will use
#include <Scheduler.h>
#include <SAMDTimerInterrupt.h>

#include "globals.h"
#include "needle.h"
#include "imu.h"
#include "ui.h"

#define USEINTERRUPTS

Needle needle;

#ifdef USEINTERRUPTS
SAMDTimer ITimer0(TIMER_TC3);

void TimerHandler0(void)
{
  // Doing something here inside ISR
    needle.run();
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
#endif  // USEINTERRUPTS

Needle::Needle(void):  stepper(AccelStepper::FULL4WIRE,PIN_STEPPER1,PIN_STEPPER2,PIN_STEPPER3,PIN_STEPPER4) {
    ;
}

void Needle::gotoangle(float angle) {
    int newpos = (int)(angle * STEPSPERREV / 360);
    int curpos = stepper.currentPosition();
    int change = newpos  - curpos;
    change = (change + STEPSPERREV / 2) % STEPSPERREV - STEPSPERREV / 2;
    //sprintf(fmtbuf,"cur=%d, new=%d, change=%d",curpos, newpos, change);
    //Serial.println(fmtbuf);
    if (abs(change)>(stepper.isRunning()?0:4))  // Only start it moving is the required change is significant
	stepper.moveTo(curpos + change);
}

void Needle::stepperadvance(void) { // Move ahead one step
    stepper.move(1);
}

void Needle::adjuststepper(void) {
    if (spinning>0 || fieldtesting) {
	return;  // No adjustments while spinning
    }
    if (!imu.isstill())
	// Unit accelerating, no update
	return;
    
    if (uiactive()) {
	gotoangle(getuipos());
    } else {
	// Adjust stepper accordinly
	static float priorHeading = 0;
	float heading = imu.getHeading();
    
	if (abs(heading-priorHeading)>=5)  {
	    priorHeading=heading;
	    gotoangle(-heading);
	}
    }
}

void Needle::setup(void) {
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

void Needle::dumpsensor(void) {
    SerialUSB.print("sensor=[");
    for (int i=0;i<360;i++) {
	SerialUSB.print(sensorVals[i]);
	if (i==359)
	    SerialUSB.print("];");
	else
	    SerialUSB.print(",");
    }
}

void Needle::sensorcheck(void) {
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


void Needle::loop(void) {
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

// Measure effect of stepper on IMU magnetometer
void Needle::stepperfield(void) {
    fieldtesting=true;
    stepper.moveTo(0);
    while (stepper.isRunning());  // wait until moved
    int sx[4],sy[4],sz[4],sx2[4],sy2[4],sz2[4];
    for (int phase=0;phase<4;phase++) { // Phases
	sx[phase]=0;
	sy[phase]=0;
	sz[phase]=0;
	sx2[phase]=0;
	sy2[phase]=0;
	sz2[phase]=0;
    }
	
    const int ncycles=20;
    sprintf(fmtbuf,"Running %d cycles: ",ncycles);
    SerialUSB.print(fmtbuf);
    for (int i=0;i<ncycles;i++) { // Cycles
	for (int phase=0;phase<4;phase++) { // Phases
	    SerialUSB.print(phase);
	    delay(100);    // Wait for magnetometer @10Hz
	    sx[phase]+=imu.rawmag_x;
	    sy[phase]+=imu.rawmag_y;
	    sz[phase]+=imu.rawmag_z;
	    sx2[phase]+=imu.rawmag_x*imu.rawmag_x;
	    sy2[phase]+=imu.rawmag_y*imu.rawmag_y;
	    sz2[phase]+=imu.rawmag_z*imu.rawmag_z;
	    stepper.move(1);   // Advance to next phase
	    //while (stepper.isRunning()); // wait until moved
	}
    }
    SerialUSB.println("done");
    float calib[4][3];
    for (int phase=0;phase<4;phase++) { // Phases
	float mx=sx[phase]*1.0/ncycles;
	float stdx=sqrt(sx2[phase]*1.0/ncycles-mx*mx);
	float my=sy[phase]*1.0/ncycles;
	float stdy=sqrt(sy2[phase]*1.0/ncycles-my*my);
	float mz=sz[phase]*1.0/ncycles;
	float stdz=sqrt(sz2[phase]*1.0/ncycles-mz*mz);
	sprintf(fmtbuf,"Phase %d: mx=%.0f += %.0f, my=%.0f += %.0f, mz=%.0f += %.0f",
		phase,mx,stdx,my,stdy,mz,stdy);
	SerialUSB.println(fmtbuf);
	calib[phase][0]=mx;
	calib[phase][1]=my;
	calib[phase][2]=mz;
    }
    // Remove the means to find the phase offsets
    for (int i=0;i<3;i++) {
	float t=0;
	for (int phase=0;phase<4;phase++) { // Phases
	    t+=calib[phase][i];
	}
	for (int phase=0;phase<4;phase++) { // Phases
	    calib[phase][i]-=t/4;
	}
    }
    for (int phase=0;phase<4;phase++) { // Phases
	sprintf(fmtbuf,"Phase %d: x:%5.0f, y:%5.0f, z:%5.0f", phase, calib[phase][0],calib[phase][1],calib[phase][2]);
	SerialUSB.println(fmtbuf);
    }
    fieldtesting=false;
}
    

void Needle::command(const char *cmd) {
    if (cmd[0]=='d' || cmd[0]=='D') {
	dumpsensor();
	SerialUSB.println("");
    } else if (cmd[0]=='s' || cmd[0]=='S') {
	spinning=2;
	SerialUSB.println("Spinning");
	stepper.move(STEPSPERREV/360);
    } else if (cmd[0]=='f' || cmd[0]=='F') {
	SerialUSB.println("Field measurement");
	stepperfield();
    }
}
