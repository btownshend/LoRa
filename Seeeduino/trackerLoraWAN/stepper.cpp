// Define a stepper and the pins it will use
#include <Scheduler.h>
#include <AccelStepper.h>

#include "globals.h"
#include "stepper.h"
#include "imu.h"

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
    // Adjust stepper accordinly
    float heading = getHeading();
    
    gotoangle(-heading);
}

void steppersetup() {
    stepper.setMaxSpeed(3600);
    stepper.setAcceleration(12000 / 100);
    SerialUSB.println("steppersetup done");
}

void stepperloop() {
    // If we have a new value, adjust stepper
    //SerialUSB.println("stepperloop");
    adjuststepper();
    stepper.run();
    delay(100);
    //SerialUSB.println("stepperloop end");
    // Check stack
    static int minstack=100000; minstack=stackcheck("Stepper",minstack);
}
