// Define a stepper and the pins it will use
#include <Scheduler.h>

#include "globals.h"
#include "needle.h"
#include "imu.h"
#include "ui.h"
#include "target.h"
#include <SAMDTimerInterrupt.h>

#define USEINTERRUPTS

Needle needle;

#ifdef USEINTERRUPTS
SAMDTimer ITimer0(TIMER_TC3);

void Needle::TimerHandler0(void)
{
  // Doing something here inside ISR
    needle.run();
}

#define TIMER0_INTERVAL_US     100
void Needle::setuptimer(void)
{
  // Interval in microsecs
  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_US, TimerHandler0))
      notice("Starting  ITimer0 OK, millis() = %d\n",millis());
  else
      error("Can't set ITimer0. Select another freq. or timer\n");
}
#endif  // USEINTERRUPTS

Needle::Needle(void):  stepper(AccelStepper::FULL4WIRE,PIN_STEPPER1,PIN_STEPPER2,PIN_STEPPER3,PIN_STEPPER4,true) {
    spinning=2;  // 2 revs at startup to find needle pos
    fieldtesting=false;
    enabled=true; // To match stepper state
    for (int i=0;i<360;i++)
	sensorVals[i]=-1;
}

void Needle::enable(void) {
    if (enabled)
	return;
    enabled=true;
    // stepper.enableOutputs(); // Executing the steps will turn on the outputs without changing them to a possible incorrect value
    //    SerialUSB.print("+");
}

void Needle::disable(void) {
    if (!enabled)
	return;
    enabled=false;
    stepper.disableOutputs();
    lastenabled=millis();
    // SerialUSB.print("-");
}

int Needle::timesinceactive(void) { // How long has it been disabled
    if (enabled)
	return 0;
    return millis()-lastenabled;
}

void Needle::move(long relative) {
    enable();
    stepper.move(relative);
}

void Needle::moveTo(long absolute) {
    enable();
    stepper.moveTo(absolute);
}

void Needle::gotoangle(float angle) {
    int newpos = (int)(angle * STEPSPERREV / 360);
    int curpos = stepper.currentPosition();
    int change = newpos%STEPSPERREV  - curpos%STEPSPERREV;
    change = (change + STEPSPERREV*5/ 2) % STEPSPERREV - STEPSPERREV / 2;
    if (change<-STEPSPERREV/2 || change>STEPSPERREV/2)
	warning("gotoangle: change=%d; newpos=%d, curpos=%d\n",change,newpos,curpos);

    //sprintf(fmtbuf,"cur=%d, new=%d, change=%d",curpos, newpos, change);
    //Serial.println(fmtbuf);
    if (abs(change)>(stepper.isRunning()?0:4))  // Only start it moving is the required change is significant
	moveTo(curpos + change);
}

void Needle::stepperadvance(void) { // Move ahead one step
    move(1);
}

void Needle::adjuststepper(void) {
    if (spinning>0 || fieldtesting) {
	return;  // No adjustments while spinning
    }
    if (lockUntil>millis()) {
	return;   // No adjustment when locked
    }
    if (lockUntil>0) {
	warning("Unlocked needle\n");
	lockUntil=0;
    }
    if (uiactive()) {
	gotoangle(getuipos());
    } else {
	// Adjust stepper accordinly
	static float priorHeading = 0;
	float heading = targets[currentTarget].getHeading()-imu.getHeading();  // Heading on compass that needle should point
    
	if (abs(heading-priorHeading)>=5)  {
	    priorHeading=heading;
	    gotoangle(heading);
	}
    }
}

void Needle::setup(void) {
    const int maxspeed=1200/2; // X27.168 spec says maximum speed is 600 deg/s -> 1200 step/s
    const int maxaccel=1200;    // maxspeed/maxaccel gives time to reach full speed
    stepper.setMaxSpeed(maxspeed);  
    stepper.setAcceleration(maxaccel);   // Acceleration tuned for the right "look" (4000 step/s/s will get it up to vmax after rotating 90 deg, but seems to lose steps then)
    float tmax=maxspeed*1.0/maxaccel;
    float degmax=maxaccel*tmax*tmax/2/2;
    notice("Stepper setup to reach max speed of %d deg/sec after %.2f sec, %.0f degrees of rotation\n", maxspeed/2, tmax, degmax);
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

int Needle::getsensor(void) {
#ifdef SEEEDUINOBOARD
    analogRead(PIN_SENSOR);
#elif defined(PROTOBOARD)
    pinMode(OS_EN_L, OUTPUT);
    digitalWrite(OS_EN_L, LOW);
    delayMicroseconds(1);
    int os1 = analogRead(OS_OUTPUT);  // With LED enabled
    digitalWrite(OS_EN_L, HIGH);
    delayMicroseconds(1);
    int os2 = analogRead(OS_OUTPUT); // LED off
    pinMode(OS_EN_L,INPUT);
    int val=os2-os1;
    static float background=15;
    if (val<30 ) {  // Background is typical ~15
	static float lastbg=0;
	background=background*0.999+val*0.001;
	if (fabs(background-lastbg)>1) {
	    lastbg=background;
	    notice("Sensor background now %.0f\n",background);
	}
    }
    val-=int(background+0.5);
    return val>0?val:0;
#else
#error BOARD not defined
#endif
}
			    
void Needle::sensorcheck(void) {
    // Check sensor

    int position=((stepper.currentPosition()-SENSOROFFSET)/(STEPSPERREV/360))%360;
    while (position<0)
	position+=360;

    static int nfound=0;
    if (sensorVals[position]==-1) {
	nfound++;
	if (nfound%10==0) {
	    verbose("Sensed %d positions\n",nfound);
	}
    }
    sensorVals[position]=getsensor(); 
    if (spinning>0 && stepper.distanceToGo()<20)
	move(20);
    
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
	if (abs(peakpos)>5)
	    warning("**** PEAK SHIFT: %d\n",peakpos);
	
	stepper.setCurrentPosition(stepper.currentPosition()-peakpos*STEPSPERREV/360+SENSOROFFSET);
	// Clear for another go
	for (int i=0;i<360;i++) 
	    sensorVals[i]=-1;   
	nfound=0;
	unsigned long toc=millis();
	verbose("Analysis took %ld msec\n",toc-tic);
    }
}


void Needle::loop(void) {
    // If we have a new value, adjust stepper
    adjuststepper();
    if (enabled && !stepper.isRunning()) {
	disable();
    }
#ifndef USEINTERRUPTS
    stepper.run();  // If not using interrupts the last step may not be executed since the stepper will be disabled below too soon
#endif
    sensorcheck();
    
    // Check stack
    static int minstack=100000; minstack=stackcheck("Stepper",minstack);
    yield();
}

// Measure effect of stepper on IMU magnetometer
void Needle::stepperfield(void) {
    fieldtesting=true;
    moveTo(0);
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
	    int mx=imu.getRawMag(0);
	    int my=imu.getRawMag(1);
	    int mz=imu.getRawMag(2);
	    sx[phase]+=mx;
	    sy[phase]+=my;
	    sz[phase]+=mz;
	    sx2[phase]+=mx*mx;
	    sy2[phase]+=my*my;
	    sz2[phase]+=mz*mz;
	    move(1);   // Advance to next phase
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
    } else if (cmd[0]=='m' || cmd[0]=='M') {
	int pos=atoi(&cmd[1]);
	lockUntil=millis()+5000;
	notice("Goto angle %d and lock needle until %ld\n", pos, lockUntil);
	gotoangle(pos);
    } else if (cmd[0]=='s' || cmd[0]=='S') {
	spinning=2;
	SerialUSB.println("Spinning");
	move(STEPSPERREV/360);
    } else if (cmd[0]=='f' || cmd[0]=='F') {
	SerialUSB.println("Field measurement");
	stepperfield();
    } else {
	warning("Expected ND, NM, NS, or NF");
    }
}
