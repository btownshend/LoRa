/******************************************************************************
u-blox_GNSS.h
u-blox GNSS Arduino
Leonardo Bispo
Mar 03, 2019
https://github.com/ldab/u-blox_GNSS

Distributed as-is; no warranty is given.
******************************************************************************/

#define GPS_RESET_N 9
#define GPS_PPS 3
#define GPS_SAFEBOOT_N 4
#define LED_PIN 13

// Enable Serial debbug on Serial UART to see registers wrote
#define GNSS_DEBUG SerialUSB

#include "ublox_GNSS.h"

#define Serial_GNSS Serial1

float lat, lon, acc;

fixType_t fix = NO_FIX;

GNSS gnss( Serial_GNSS );

void setup() {
  // put your setup code here, to run once:
  pinMode(GPS_RESET_N, OUTPUT); // hold in reset
  digitalWrite(GPS_RESET_N, LOW);
  pinMode(GPS_SAFEBOOT_N, OUTPUT);
  digitalWrite(GPS_SAFEBOOT_N, HIGH); //safeboot off

  SerialUSB.begin( 115200 );
  delay(3000);
  SerialUSB.println("Basic started");
  Serial_GNSS.begin( 9600 );

  // Release reset
  pinMode(GPS_RESET_N, OUTPUT);
  digitalWrite(GPS_RESET_N, HIGH);
  SerialUSB.println("Released reset");
  delay(1000);
  DBG("DBG Test\n");
  SerialUSB.print("&gnss=0x");
  SerialUSB.println((int)&gnss,HEX);
  SerialUSB.println("Starting init");
  if( gnss.init(  ) )
  {
    SerialUSB.println("\nGNSS initialized.");
  }
  else
  {
    SerialUSB.println("\nFailed to initialize GNSS module.");
  }

}

void loop()
{
  // Get coordinates with minimum 100m accuracy;
  SerialUSB.println("Get location");

  if ( gnss.getCoodinates(lon, lat, fix, acc, 100) == 0) 
  {
    SerialUSB.println("Failed to get coordinates, check signal, accuracy required or wiring");
  }

  SerialUSB.println("\nHere you are, lon:" + String(lon, 7) +" lat:" + String(lat, 7));
  SerialUSB.println("calculated error: " + String(acc) + "m");
  
  SerialUSB.println("\nOr try the following link to see on google maps:");
  SerialUSB.println(String("https://www.google.com/maps/search/?api=1&query=") + String(lat,7) + "," + String(lon,7));

  delay(50000);

}
