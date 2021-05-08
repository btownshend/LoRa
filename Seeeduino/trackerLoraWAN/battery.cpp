#include <Arduino.h>
#include "battery.h"

// battey of Seeeduino LoRaWAN
const int pin_battery_status  = A5;
const int pin_battery_voltage = A4;

unsigned short batteryvoltage() {
  // Not clear what pulling down the status pin is supposed to achieve
  pinMode(pin_battery_status , OUTPUT);
  digitalWrite(pin_battery_status , LOW);
  delay(1);  /// This may cause issues with losing serial chars -- @115,200 1ms = 12 char (should be OK when using Scheduler)
  unsigned short battery = (analogRead(pin_battery_voltage) * 3300 * 11) >> 10;
  pinMode(pin_battery_status , INPUT);

  SerialUSB.print("Battery: ");
  SerialUSB.println(battery);
  return battery;
}

unsigned char batterystatus() {
  // 0-charging, 1-onbattery
  return digitalRead(pin_battery_status);
}


void batterysetup() {
      pinMode(pin_battery_status, INPUT);
}
