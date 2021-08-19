#define GPS_RESET_N 9
#define GPS_PPS 3
#define GPS_SAFEBOOT_N 4
#define LED_PIN 13


void setup() {
  // put your setup code here, to run once:
  pinMode(GPS_RESET_N, OUTPUT); // hold in reset
  digitalWrite(GPS_RESET_N, LOW);
  pinMode(GPS_SAFEBOOT_N, OUTPUT);
  digitalWrite(GPS_SAFEBOOT_N, HIGH); //safeboot off

  SerialUSB.begin(115200);
  Serial1.begin(9600);
  delay(5000);

  SerialUSB.println("starting test");
  delay(1000);
  SerialUSB.println("starting gps");

  pinMode(GPS_RESET_N, OUTPUT);
  digitalWrite(GPS_RESET_N, HIGH);

  pinMode(LED_PIN, OUTPUT);
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
  }
  delay(500);
}
