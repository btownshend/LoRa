#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <Arduino.h>

extern char fmtbuf[];

extern unsigned int stackcheck(const char *module, unsigned int minstack);

extern Uart Serial3;   // External Serial port on LoRaWAN board (D4/D5).

//#define EXTERNALGPS   // Define to use GPS connected to external serial port
//#define EXTERNALBLE // BLE Connected
//#define EXTERNALLORA

#if defined(EXTERNALGPS) && defined(EXTERNALBLE)
Conflict -- only one external Serial port
#endif

// Define one of the following to set the IMU type
#define IMU_9250
//#define IMU_AK

#ifdef EXTERNALGPS
#define SerialGPS Serial3   // External serial port
#else
#define SerialGPS Serial2   // Built-in GPS on LoraWan board
#endif

#ifdef EXTERNALBLE
#define SerialBLE Serial3   // External serial port
#endif

#ifdef EXTERNALLORA
#define SerialLoRa Serial3
#else
#define SerialLoRa Serial1
#endif

// Pin usage
// A0,A1,A2,A3 are also on Grove connectors
#define PIN_SENSOR A0  // Sensor for IR detector for needle
// A4,A5 are used for battery charge monitoring

// D0,D1 are serial to LoRa module
#define PIN_STEPPER1  2  // D2,D3 are also connected to LoRa CTS/RTS (unused?)
#define PIN_STEPPER2  3
#define PIN_SERIAL3_TX       4   // PA14 SERCOM2:pad2  (D4 on connector)
#define PIN_SERIAL3_RX       5  // PA15 SERCOM2:pad3  (D5 on connector)
#define PIN_STEPPER3  6
#define PIN_STEPPER4  7  // D7 is also connected to GPS PPS output (unused?)
// D8,D9 are LoRa RST and DFU
// D10-D13 are SPI (unused?)

#endif /* _GLOBALS_H_ */
