extern char fmtbuf[];

extern unsigned int stackcheck(const char *module, unsigned int minstack);

extern Uart Serial3;   // External Serial port on LoRaWAN board (D4/D5).

#define EXTERNALGPS 1   // Define to use GPS connected to external serial port

#ifdef EXTERNALGPS
#define SerialGPS Serial3   // External serial port
#else
#define SerialGPS Serial2   // Built-in GPS on LoraWan board
#endif

