#include "wiring_private.h" // pinPeripheral() function

#define PIN_SERIAL3_RX       (5ul)   // PA15 SERCOM2:pad3  (D5 on connector)
#define PIN_SERIAL3_TX       (4ul)   // PA14 SERCOM2:pad2  (D4 on connector)
Uart Serial3( &sercom2, PIN_SERIAL3_RX, PIN_SERIAL3_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2 ) ;

void SERCOM2_Handler() {
  Serial3.IrqHandler();
}

void setup() {
  Serial.begin(115200);
  delay(100);
  //Serial3.begin(230400);  // For e-ink display
  Serial3.begin(9600);  // For GPS or BLE
  //Serial3.begin(115200);  // For LoraWan (after baud adjusted)
  // Change PA14,PA15 to SERCOM (mode C)
  pinPeripheral(PIN_SERIAL3_RX, PIO_SERCOM);
  pinPeripheral(PIN_SERIAL3_TX, PIO_SERCOM);
  Serial.println("sercomtest ready");
}

void loop() {
  while (Serial3.available())
  {
    Serial.write(Serial3.read());
  }
  while (Serial.available())
  {
    Serial3.write(Serial.read());
  }
}
