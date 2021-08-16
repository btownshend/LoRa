#define GPS_RESET_N 9
#define GPS_PPS 3
#define GPS_SAFEBOOT_N 4
#define LED_PIN 13

#define BATT_MEAS_EN A0
#define BATT_MEAS A5


void setup() {
  // put your setup code here, to run once:
  pinMode(GPS_RESET_N, OUTPUT); // hold in reset
  digitalWrite(GPS_RESET_N, LOW);
  pinMode(GPS_SAFEBOOT_N, OUTPUT);
  digitalWrite(GPS_SAFEBOOT_N, HIGH); //safeboot off

  SerialUSB.begin(115200);
  Serial1.begin(9600);
  //while(!SerialUSB);
  delay(5000);

  SerialUSB.println("starting test");



  delay(1000);
  SerialUSB.println("starting gps");

  pinMode(GPS_RESET_N, OUTPUT);
  digitalWrite(GPS_RESET_N, HIGH);

  pinMode(LED_PIN, OUTPUT);

  pinMode(BATT_MEAS_EN, OUTPUT);
  digitalWrite(BATT_MEAS_EN, LOW);
  batteryCheck();
}

void batteryCheck()
{
  digitalWrite(BATT_MEAS_EN, HIGH);
  delay(100);

  SerialUSB.print((6600 * analogRead(BATT_MEAS)) / 1024);
  SerialUSB.println(" mV");
  SerialUSB.println(analogRead(BATT_MEAS));
  digitalWrite(BATT_MEAS_EN, LOW);
}

void loop()
{
  //delay(1000);
  //SerialUSB.println("tick");
  digitalWrite(LED_PIN, digitalRead(GPS_PPS));

  if (Serial1.available() > 0)
  {
    //SerialUSB.println("available");
    while (Serial1.available() > 0)
      SerialUSB.write(Serial1.read());
    batteryCheck();
  }
  delay(500);
}
