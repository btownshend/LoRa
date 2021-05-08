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
    // roll/pitch in radian
    double roll = atan2((float)acc_y, (float)acc_z);
    double pitch = atan2(-(float)acc_x, sqrt((float)acc_y * acc_y + (float)acc_z * acc_z));

    double Xheading = mag_x * cos(pitch) + mag_y * sin(roll) * sin(pitch) + mag_z * cos(roll) * sin(pitch);
    double Yheading = mag_y * cos(roll) - mag_z * sin(pitch);

    const double declination = -6;   // Magnetic declination
    double heading = 180 + 57.3 * atan2(Yheading, Xheading) + declination;
    //double heading = 180 + 57.3 * atan2(mag_y, mag_x) + declination;

    double field = sqrt(1.0 * mag_x * mag_x + 1.0 * mag_y * mag_y + 1.0 * mag_z * mag_z);

    gotoangle(heading);

    static unsigned long lastdbg = 0;

    if (millis() - lastdbg > 1000 ) {
	sprintf(fmtbuf, "Roll: %.0f, Pitch: %.0f, Heading: %.0f, Field: %.0f", (roll * 57.3),( pitch * 57.3),heading, field);
	SerialUSB.println(fmtbuf);
	lastdbg = millis();
    }
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
