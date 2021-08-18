#include "globals.h"
#include "battery.h"

#ifdef SEEEDUINOBOARD
unsigned short batteryvoltage() {
  // Not clear what pulling down the status pin is supposed to achieve
  pinMode(pin_battery_status , OUTPUT);
  digitalWrite(pin_battery_status , LOW);
  delay(1);  /// This may cause issues with losing serial chars -- @115,200 1ms = 12 char (should be OK when using Scheduler)
  unsigned short battery = (analogRead(pin_battery_voltage) * 3300 * 11) >> 10;
  pinMode(pin_battery_status , INPUT);

  notice("Battery: %d\n",battery);
  return battery;
}

unsigned char batterystatus() {
  // 0-charging, 1-onbattery
  return digitalRead(pin_battery_status);
}


void batterysetup() {
      pinMode(pin_battery_status, INPUT);
}
#elif defined(PROTOBOARD)
static unsigned short battery=0;

unsigned short batteryvoltage() {
  // Enable the battery measurement
  digitalWrite(PIN_BATT_MEAS_EN , HIGH);
  delay(1);  /// This may cause issues with losing serial chars -- @115,200 1ms = 12 char (should be OK when using Scheduler)
  int batRead=analogRead(PIN_BATT_MEAS);
  //digitalWrite(PIN_BATT_MEAS_EN , LOW);
  battery = (batRead * 6600) >> 10;
  notice("Battery: analog: %d, mV: %d\n",batRead, battery);
  return battery;
}

unsigned char batterystatus() {
  // 0-charging, 1-onbattery
    return battery<4000;
}


void batterysetup() {
      pinMode(PIN_BATT_MEAS_EN, OUTPUT);
      digitalWrite(PIN_BATT_MEAS_EN,LOW);
}
#else
#error Board not defined
#endif
