#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <Arduino.h>
#include "log.h"

#define PROTOBOARD 1
// #define SEEEDUINOBOARD 1

extern char fmtbuf[];

extern unsigned int stackcheck(const char *module, unsigned int minstack);


#ifdef SEEEDUINOBOARD
// -----------------  Seeeduino board ----------------
// Define one of the following to set the IMU type
#define IMU_9250
//#define IMU_AK

#define SerialGPS Serial2   // Built-in GPS on LoraWan board
#define SerialLoRa Serial1

// Pin usage
// A0,A1,A2,A3 are also on Grove connectors
#define PIN_SENSOR A0  // Sensor for IR detector for needle
// A4,A5 are used for battery charge monitoring

// D0,D1 are serial to LoRa module
#define PIN_STEPPER1  2  // D2,D3 are also connected to LoRa CTS/RTS (unused?)
#define PIN_STEPPER2  3
#define PIN_SERIALEXT_TX       4   // PA14 SERCOM2:pad2  (D4 on connector)
#define PIN_SERIALEXT_RX       5  // PA15 SERCOM2:pad3  (D5 on connector)
#define PIN_STEPPER3  6
#define PIN_STEPPER4  7  // D7 is also connected to GPS PPS output (unused?)
// D8,D9 are LoRa RST and DFU
// D10-D13 are SPI (unused?)

// battery of Seeeduino LoRaWAN
const int pin_battery_status  = A5;
const int pin_battery_voltage = A4;

#elif defined(PROTOBOARD)
// -----------------  New custom board ----------------
#define SerialGPS Serial1
#define SerialLoRa Serial
#define IMU_9250

// Pin usage
#define OS_OUTPUT 17 // analog read
#define OS_EN_L  18 // enable LED emitter when low

#define MOTOR_1A 12
#define MOTOR_1B 10
#define MOTOR_2A 11
#define MOTOR_2B 5

#define PIN_STEPPER1  MOTOR_1A
#define PIN_STEPPER2  MOTOR_1B
#define PIN_STEPPER4  MOTOR_2B
#define PIN_STEPPER3  MOTOR_2A

// Grove connector serial
#define PIN_SERIALEXT_RX 24 
#define PIN_SERIALEXT_TX 23

// battery of Seeeduino LoRaWAN
const int PIN_BATT_MEAS = A5;
const int PIN_BATT_MEAS_EN = A0;

#else
// --------------------------------------------------------------
#error Neither PROTOBOARD nor SEEEDUINOBOARD defined
#endif

//#define EXTERNALGPS   // Define to use GPS connected to external serial port
#define EXTERNALBLE // BLE Connected
//#define EXTERNALLORA

#if defined(EXTERNALGPS) && defined(EXTERNALBLE)
Conflict -- only one external Serial port
#endif

extern Uart SerialExt;   // External Serial port (Grove connector on Proto, D4/D5 on Seeeduino)

#ifdef EXTERNALGPS
#define SerialGPS SerialExt   // External serial port
#endif

#ifdef EXTERNALBLE
#define SerialBLE SerialExt   // External serial port
#endif

#ifdef EXTERNALLORA
#define SerialLoRa SerialExt
#endif

#endif /* _GLOBALS_H_ */
