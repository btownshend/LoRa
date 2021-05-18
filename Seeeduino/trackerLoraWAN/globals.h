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
//#define IMU_9250
#define IMU_AK

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

#define SENSORPIN A0  // Sensor for IR detector for needle

#endif /* _GLOBALS_H_ */
