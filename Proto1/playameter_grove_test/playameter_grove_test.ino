#include "wiring_private.h" // pinPeripheral() function

#define PIN_SERIALEXT_RX 24 
#define PIN_SERIALEXT_TX 23

// Add SerialExt using SERCOM4 (Grove connector)
Uart SerialExt( &sercom4, PIN_SERIALEXT_RX, PIN_SERIALEXT_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2 ) ;
int nint=0;

void SERCOM4_Handler() {
  SerialExt.IrqHandler();
  SerialUSB.print("!");
}


void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  SerialExt.begin(9600);

  // Change SerialExt pins to SERCOM (mode C) (AFTER SerialExt.begin)
  pinPeripheral(PIN_SERIALEXT_RX, PIO_SERCOM);
  pinPeripheral(PIN_SERIALEXT_TX, PIO_SERCOM);

  delay(5000);
  SerialUSB.println("starting grove port test");
}

void loop()
{
  while (SerialExt.available() > 0) {
      SerialUSB.print("<");
      SerialUSB.write(SerialExt.read());
  }
  while (SerialUSB.available() > 0) {
      SerialUSB.print(">");
      SerialExt.write(SerialUSB.read());
  }
  if (nint>0) {
      SerialUSB.println(nint);
      nint=0;
  }
}
