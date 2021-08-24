#include <Scheduler.h>

#define E5_RESET 16

#if 0
// Add Serial2 using SERCOM5
#include "wiring_private.h" // pinPeripheral() function

#define PIN_SERIAL5_TX       30   // PB22 SERCOM5:pad2
#define PIN_SERIAL5_RX       31  // PB23 SERCOM5:pad3

Uart Serial2( &sercom5, PIN_SERIAL5_RX, PIN_SERIAL5_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2 ) ;

void SERCOM5_Handler() {
    Serial2.IrqHandler();
} 
#endif

#define SerialE5 Serial   // Predefined in variant.cpp as SERCOM5 on pins 30(Tx),31(Rx)

void setup()
{
    pinMode(E5_RESET,OUTPUT);
    digitalWrite(E5_RESET,HIGH);
    // SerialE5.begin(9600);   // Assuming baud rate already changed, default is 9600 for new boards
    SerialE5.begin(115200);   // Assuming baud rate already changed, default is 9600 for new boards
    SerialUSB.begin(115200);
    for (int i = 0; i < 5; i++) {
	SerialUSB.print(i, DEC);
	SerialUSB.print("...");
	delay(1000);
    }
    SerialUSB.println("LoraSerialTest Ready");
    Scheduler.startLoop(loop2);
}

void loop() {
    while (SerialE5.available())
	{
	    SerialUSB.write(SerialE5.read());
	}
    yield();
}

void loop2() {
    while (SerialUSB.available())
	{
	    char x=SerialUSB.read();
	    if (x=='@') {
		// Reset
		digitalWrite(E5_RESET, LOW);
		delay(100);
		digitalWrite(E5_RESET,HIGH);
		SerialUSB.println("RESET");
	    } else
		SerialE5.write(x);
	}
    yield();
}
