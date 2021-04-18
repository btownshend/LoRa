// battey of Seeeduino LoRaWAN
 
const int pin_battery_status  = A5;
const int pin_battery_voltage = A4;
 
void setup() {
    SerialUSB.begin(115200);
    pinMode(pin_battery_status, INPUT);
}
 
void loop() {
 
    int a = analogRead(pin_battery_voltage);
    float v = a/1023.0*3.3*11.0;        // there's an 1M and 100k resistor divider
    SerialUSB.print(v, 2);
    SerialUSB.print('\t');
    SerialUSB.println(digitalRead(pin_battery_status));
 
    delay(5000);
}
