#define LED_PIN 13

#define BATT_MEAS_EN A0
#define BATT_MEAS A5


void setup() {
  // put your setup code here, to run once:

  SerialUSB.begin(115200);
  delay(5000);

  SerialUSB.println("starting test");

  pinMode(LED_PIN, OUTPUT);

  pinMode(BATT_MEAS_EN, OUTPUT);
  digitalWrite(BATT_MEAS_EN, LOW);
  batteryCheck();
}

void batteryCheck()
{
  digitalWrite(BATT_MEAS_EN, HIGH);
  // delayMicroseconds(1);
  int rd=analogRead(BATT_MEAS);
  int v=(6600 * rd) / 1024;
  SerialUSB.print(rd);
  SerialUSB.print(" -> ");
  SerialUSB.print(v);
  SerialUSB.println(" mV");
  digitalWrite(BATT_MEAS_EN, LOW);
  digitalWrite(LED_PIN,(v>3800)?HIGH:LOW);
}

void loop()
{
    delay(1000);
    batteryCheck();	
}
