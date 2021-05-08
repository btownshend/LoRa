// Random.pde
// -*- mode: C++ -*-
//
// Make a single stepper perform random changes in speed, position and acceleration
//
// Copyright (C) 2009 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5


const int STEPSPERREV = 720;

void setup()
{
  Serial.begin(115200);
  //stepper.setMaxSpeed(3600);
  //stepper.setAcceleration(9600);
  stepper.setMaxSpeed(3600);
  stepper.setAcceleration(12000 / 100);
}

void gotoangle(int angle) {
  int newpos = angle * STEPSPERREV / 360;
  int curpos = stepper.currentPosition();
  int change = newpos  - curpos;
  change = (change + STEPSPERREV / 2) % STEPSPERREV - STEPSPERREV / 2;
  char fmtbuf[100];
  sprintf(fmtbuf, "cur=%d, new=%d, change=%d", curpos, newpos, change);
  Serial.println(fmtbuf);
  stepper.moveTo(curpos + change);
}

void loop()
{
  static int angle = 0;
  static int delta = 90;
  static int lasttogo = 0;

  int togo=stepper.distanceToGo();
  if (togo == 0)
  {
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
  stepper.run();
}
