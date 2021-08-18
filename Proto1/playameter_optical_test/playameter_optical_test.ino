#define OS_OUTPUT 17 // analog read
#define OS_EN_L  18 // enable LED emitter when low


void setup() {
  // put your setup code here, to run once:
  pinMode(OS_EN_L, OUTPUT);
  digitalWrite(OS_EN_L, LOW);

  digitalWrite(OS_OUTPUT, LOW);
  pinMode(OS_OUTPUT,INPUT);

  SerialUSB.begin(115200);
  delay(5000);
  SerialUSB.println("starting test");
}
#define PERIOD 500

void loop()
{
  pinMode(OS_EN_L, OUTPUT);
  digitalWrite(OS_EN_L, LOW);
  delay(1);
  int os1 = analogRead(OS_OUTPUT);  // With LED enabled
  digitalWrite(OS_EN_L, HIGH);
  delay(1);
  int os2 = analogRead(OS_OUTPUT); // LED off
  pinMode(OS_EN_L,INPUT);

  SerialUSB.print(os1);
  SerialUSB.print(",");
  SerialUSB.print(os2);
  SerialUSB.print(" -> ");
  SerialUSB.println(os2-os1);

  delay(PERIOD);
}
