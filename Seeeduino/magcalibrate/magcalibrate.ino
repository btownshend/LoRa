#include "AK09918.h"
#include "ICM20600.h"
#include <Wire.h>

AK09918_err_type_t err;
AK09918 ak09918;
ICM20600 icm20600(true);


void setup() {
  Serial.begin(9600);
  delay(3000);
  Serial.println("setup");

  // Enable Grove connectors
  digitalWrite(38, HIGH);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  err = ak09918.initialize();
  if (err != AK09918_ERR_OK) {
    Serial.print("Error initializing AK09918: 0x");
    Serial.println(err, HEX);
  }

  err = ak09918.selfTest();
  if (err != AK09918_ERR_OK) {
    Serial.print("Error in selfTest of AK09918: 0x");
    Serial.println(err, HEX);
  }

  icm20600.initialize();
  ak09918.switchMode(AK09918_POWER_DOWN);
  ak09918.switchMode(AK09918_CONTINUOUS_100HZ);

  err = ak09918.isDataReady();
  while (err != AK09918_ERR_OK) {
    if (err == AK09918_ERR_NOT_RDY)
      Serial.println("Waiting Sensor");
    else if (err == AK09918_ERR_READ_FAILED)
      Serial.println("AK09918_ERR_READ_FAILED");
    else {
      Serial.print("Unexpected error: 0x");
      Serial.println(err, HEX);
    }
    delay(100);
    err = ak09918.isDataReady();
  }

  Serial.println("ok");
}

void loop() {
  if (ak09918.isDataReady() == AK09918_ERR_OK) {
    // get acceleration
    int ax = icm20600.getAccelerationX();
    int ay = icm20600.getAccelerationY();
    int az = icm20600.getAccelerationZ();

    ax *= 8;
    ay *= 8;
    az *= 8;

    short gx = icm20600.getGyroscopeX();
    short gy = icm20600.getGyroscopeY();
    short gz = icm20600.getGyroscopeZ();


    int32_t mx, my, mz;
    err = ak09918.getRawData(&mx, &my, &mz);
    mx = (mx * 15 / 10);
    my = (my * 15 / 10);
    mz = (mz * 15 / 10);

    if (err !=  AK09918_ERR_OK) {
      Serial.print("getRawData err: 0x");
      Serial.println(err, HEX);
      return;
    }


    Serial.print("Raw:");
    Serial.print(ax);
    Serial.print(',');
    Serial.print(ay);
    Serial.print(',');
    Serial.print(az);
    Serial.print(',');
    Serial.print(gx);
    Serial.print(',');
    Serial.print(gy);
    Serial.print(',');
    Serial.print(gz);
    Serial.print(',');
    Serial.print(mx);
    Serial.print(',');
    Serial.print(my);
    Serial.print(',');
    Serial.print(mz);
    Serial.println();
  }
  delay(10);
}
