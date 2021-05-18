#include <SAMDTimerInterrupt.h>
#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
#define USEINTERRUPTS


const int STEPSPERREV = 720;
const int SENSORPIN = A0;
char fmtbuf[100];
unsigned long nintr=0;

#ifdef USEINTERRUPTS
    SAMDTimer ITimer0(TIMER_TC3);

void TimerHandler0(void)
{
  // Doing something here inside ISR
    stepper.run();
    nintr+=1;
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


void setup()
{
    Serial.begin(115200);
    delay(2000);
    SerialUSB.println("steppertest");
    
    const int maxspeed=1200; // X27.168 spec says maximum speed is 600 deg/s -> 1200 step/s
    const int maxaccel=12000;    // maxspeed/maxaccel gives time to reach full speed
    stepper.setMaxSpeed(maxspeed);  
    stepper.setAcceleration(maxaccel);   // Acceleration tuned for the right "look" (4000 step/s/s will get it up to vmax after rotating 90 deg, but seems to lose steps then)
    float tmax=maxspeed*1.0/maxaccel;
    float degmax=maxaccel*tmax*tmax/2/2;
    sprintf(fmtbuf,"Stepper setup to reach max speed of %d deg/sec after %.2f sec, %.0f degrees of rotation", maxspeed/2, tmax, degmax);
    SerialUSB.println(fmtbuf);

    pinMode(SENSORPIN, INPUT);
#ifdef USEINTERRUPTS
    setuptimer();
#endif
}

void gotoangle(int angle) {
    int newpos = angle * STEPSPERREV / 360;
    int curpos = stepper.currentPosition();
    int change = newpos  - curpos;
    change = (change + STEPSPERREV / 2) % STEPSPERREV - STEPSPERREV / 2;
    //sprintf(fmtbuf, "cur=%d, new=%d, change=%d", curpos, newpos, change);
    //Serial.println(fmtbuf);
    stepper.moveTo(curpos + change);
}

// Get current reading from position sensor
int getsensor() {
    return analogRead(SENSORPIN);
}

void test1()
{
    static int angle = 0;
    static int delta = 90;
    static int lasttogo = 0;

    int togo=stepper.distanceToGo();
    if (togo == 0) {
	Serial.println(",");
	delay(2000);
	angle += delta;
	if (angle > 360)
	    delta = -delta;
	if (angle < 0)
	    delta = -delta;
	gotoangle(angle);
    } else if (togo!= lasttogo) {
	Serial.print(stepper.distanceToGo());
	Serial.print(",");
	lasttogo=togo;
    }
}

void test2() {
    static int angle = 0;

    int togo=stepper.distanceToGo();
    if (togo == 0) {
	angle+=107;
	delay(1000);
	gotoangle(angle);
    }
    int s=getsensor();
    int pos = stepper.currentPosition()%STEPSPERREV;
    static int lasts=0;
    static int lastpos=0;
    if (s!=lasts || pos!=lastpos) {
	Serial.print(millis());
	Serial.print(",");
	Serial.print(pos);
	Serial.print(",");
	Serial.println(s);
	lasts=s;
	lastpos=pos;
    }
}

void test3() {
    // Locate index position
    static int state = 0;   // 0-reset, 1-full scan seek, 2-localize
    static long peakposition=0;
    static int peaklevel=0;

    if (stepper.distanceToGo() ==0) {
	if (state==0) {
	    SerialUSB.println("Starting full scan");
	    stepper.setCurrentPosition(0);
	    stepper.moveTo(STEPSPERREV*11/10);
	    peakposition=0;
	    peaklevel=0;
	    state=1;
	} else if (state==1) {
	    SerialUSB.println("Finished full scan");
	    stepper.moveTo(peakposition);
	    state=2;
	} else if (state==2) {
	    delay(5000);
	    SerialUSB.println("Restarting");
	    stepper.setCurrentPosition(50);
	    state=0;
	} else {
	    ; // Bad state
	}
    }
    int s=getsensor();
    if (s>peaklevel) {
	long priorposition = peakposition;
	peakposition = stepper.currentPosition();
	peaklevel=s;
	if (peaklevel>100 && peakposition!=priorposition) {
	    sprintf(fmtbuf,"New peak %d at %d steps", peaklevel, peakposition);
	    Serial.println(fmtbuf);
	}
    }
}

void test4() {
    const long maxval=STEPSPERREV*10;
    if (stepper.distanceToGo() == 0) {
	static unsigned long lastnintr=0;
	static unsigned long lasttime=0;
	sprintf(fmtbuf,"nintr=%d, dt=%d, period=%.0f usec", nintr-lastnintr,millis()-lasttime,(millis()-lasttime)*1000.0/(nintr-lastnintr));
	lastnintr=nintr; lasttime=millis();
	SerialUSB.println(fmtbuf);
	if (stepper.currentPosition() <= 0) {
	    stepper.moveTo(maxval);
	    SerialUSB.println("fwd");
	} else {
	    stepper.moveTo(0);
	    SerialUSB.println("rev");
	}
    }
    int s=getsensor();
    static int lastPos=0;
    static int lastTime=0;
    int curPos=stepper.currentPosition();
    int change = abs(lastPos-curPos);
    if (s>300 && change>STEPSPERREV/4) {
	int deltat=millis()-lastTime;
	int stepspersec = change*1000/deltat;
	SerialUSB.print(curPos);
	SerialUSB.print(",");
	SerialUSB.println(stepspersec);
	lastTime+=deltat;
	lastPos=curPos;
    }
}

void loop() {
    const int runtest = 4;
    if (runtest==1)
	test1();
    else if (runtest==2)
	test2();
    else if (runtest==3)
	test3();
    else if (runtest==4)
	test4();
    else
	SerialUSB.println("Bad test");
    delay(7);
#ifndef USEINTERRUPTS
    stepper.run();
#endif
}
