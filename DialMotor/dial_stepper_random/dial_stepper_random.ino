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

int pos = 0;
int pause=0;
int wiggle=20;

void setup()
{
  Serial.begin(115200);
  //stepper.setMaxSpeed(3600);
  //stepper.setAcceleration(9600);
  stepper.setMaxSpeed(3600);
  stepper.setAcceleration(12000/10);
}

void loop()
{
  if (stepper.distanceToGo() == 0)
  {
    // Random change to speed, position and acceleration
    // Make sure we dont get 0 speed or accelerations
    delay(100);
    if (pause<20) {
      pause+=1;
      wiggle=-wiggle;
    } else {
      pause=0;
      //pos += (rand() % 2400) - 1200;
      //wiggle= (rand() % 50);
      pos+=720; wiggle=0;
      Serial.println("");
    }

    stepper.moveTo(pos+wiggle);
    Serial.print(pos+wiggle);
    Serial.print(",");
  }
  stepper.run();
}
