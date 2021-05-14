#ifndef _GLOBALS_H_
#define _GLOBALS_H_
extern char fmtbuf[];

extern unsigned int stackcheck(const char *module, unsigned int minstack);

extern Uart Serial3;   // External Serial port on LoRaWAN board (D4/D5).

#define EXTERNALGPS   // Define to use GPS connected to external serial port

// Define one of the following to set the IMU type
#define IMU_9250
//#define IMU_AK

#ifdef EXTERNALGPS
#define SerialGPS Serial3   // External serial port
#else
#define SerialGPS Serial2   // Built-in GPS on LoraWan board
#endif

#endif /* _GLOBALS_H_ */
