#include <Scheduler.h>

void setup()
{
  Serial1.begin(115200);   // Assuming baud rate already changed, default is 9600 for new boards
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
  while (Serial1.available())
  {
    SerialUSB.write(Serial1.read());
  }
  yield();
}

void loop2() {
  while (SerialUSB.available())
  {
    Serial1.write(SerialUSB.read());
  }
  yield();
}
