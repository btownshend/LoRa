#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5


const int STEPSPERREV = 720;
const int SENSORPIN = A0;
char fmtbuf[100];

void setup()
{
    Serial.begin(115200);
    stepper.setMaxSpeed(3600);
    stepper.setAcceleration(12000 / 100);
    pinMode(SENSORPIN, INPUT);
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

void loop() {
    const int runtest = 3;
    if (runtest==1)
	test1();
    else if (runtest==2)
	test2();
    else if (runtest==3)
	test3();
    
    stepper.run();
}
