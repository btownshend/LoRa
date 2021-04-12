void setup()
{
    Serial1.begin(9600);
    SerialUSB.begin(115200);
    for (int i=0;i<5;i++) {
      SerialUSB.print(i,DEC);
      SerialUSB.print("...");
      delay(1000);
    }
    SerialUSB.println("LoraSerialTest Ready");
}
 
void loop()
{
    while(Serial1.available())
    {
        SerialUSB.write(Serial1.read());
    }
    while(SerialUSB.available())
    {
        Serial1.write(SerialUSB.read());
    }
}
