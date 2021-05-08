#include "AK09918.h"
#include "ICM20600.h"
#include <Wire.h>
// If there's a complie error in I2Cdev.cpp, need to add #define BUFFER_LENGTH 32 in I2Cdev.h in library

#include "imu.h"

extern char fmtbuf[];

// 9DOF
short acc_x, acc_y, acc_z;
short gyro_x, gyro_y, gyro_z;
short mag_x, mag_y, mag_z;
bool have9dof = true;

bool update9DOF() {
  static unsigned long lasttime = 0;

  if (millis() - lasttime < 100) {
    return false;
  }
  lasttime = millis();

  if (!have9dof)
    return false;

  static bool initialized = false;
  static AK09918 ak09918;
  static ICM20600 icm20600(true);
  AK09918_err_type_t err;
  static short mag_xmin = -1000, mag_xmax = 1000, mag_ymin = -1000, mag_ymax = 1000, mag_zmin = -1000, mag_zmax = 1000;

  if (!initialized) {
    // Enable Grove connectors
    digitalWrite(38, HIGH);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    err = ak09918.initialize();
    if (err != AK09918_ERR_OK) {
      Serial.print("Error initializing AK09918: 0x");
      Serial.println(err, HEX);
      have9dof=false;
      return false;
    }

    err = ak09918.selfTest();
    if (err != AK09918_ERR_OK) {
      Serial.print("Error in selfTest of AK09918: 0x");
      Serial.println(err, HEX);
      have9dof=false;
      return false;
    }

    icm20600.initialize();
    ak09918.switchMode(AK09918_POWER_DOWN);
    ak09918.switchMode(AK09918_CONTINUOUS_100HZ);
    delay(100);
    SerialUSB.println("9DOF Initialized");
    initialized = true;
  }
  // Module mounted on side, so x=mz, y=mx, z=-my;
  acc_x = icm20600.getRawAccelerationX();
  acc_z = -icm20600.getRawAccelerationY();
  acc_y = icm20600.getRawAccelerationZ();
  gyro_x = icm20600.getRawGyroscopeX();
  gyro_z = -icm20600.getRawGyroscopeY();
  gyro_y = icm20600.getRawGyroscopeZ();
  int32_t x, y, z;
  err = ak09918.getRawData(&x, &y, &z);  // raw data is in units of 0.15uT
  //mag_x = x; mag_y = y; mag_z = z;
  mag_x = x; mag_y = z; mag_z = -y;  // Module is mounted on side

  bool changed = false;
  if (mag_x < mag_xmin) {
    mag_xmin = mag_x;
    changed = true;
  }
  if (mag_x > mag_xmax) {
    mag_xmax = mag_x;
    changed = true;
  }
  if (mag_y < mag_ymin) {
    mag_ymin = mag_y;
    changed = true;
  }
  if (mag_y > mag_ymax) {
    mag_ymax = mag_y;
    changed = true;
  }
  if (mag_z < mag_zmin) {
    mag_zmin = mag_z;
    changed = true;
  }
  if (mag_z > mag_zmax) {
    mag_zmax = mag_z;
    changed = true;
  }
  static float scale_x = 1, scale_y = 1, scale_z = 1;
  static int offset_x = 0, offset_y = 0, offset_z = 0;
  if (changed) {
    sprintf(fmtbuf, "New mag range: [%d,%d]; [%d,%d]; [%d,%d]", mag_xmin, mag_xmax, mag_ymin, mag_ymax, mag_zmin, mag_zmax);
    SerialUSB.println(fmtbuf);
    offset_x = (mag_xmax + mag_xmin) / 2;
    offset_y = (mag_ymax + mag_ymin) / 2;
    offset_z = (mag_zmax + mag_zmin) / 2;

    float avg_delta_x = (mag_xmax - mag_xmin) / 2.0;
    float avg_delta_y = (mag_ymax - mag_ymin) / 2.0;
    float avg_delta_z = (mag_zmax - mag_zmin) / 2.0;

    float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0;

    scale_x = avg_delta / avg_delta_x;
    scale_y = avg_delta / avg_delta_y;
    scale_z = avg_delta / avg_delta_z;
    sprintf(fmtbuf, "Offset=%d,%d,%d Scale=%.2f,%.2f,%.2f", offset_x, offset_y, offset_z, scale_x, scale_y, scale_z);
    SerialUSB.println(fmtbuf);
  }
  mag_x = (short)((mag_x - offset_x) * scale_x);
  mag_y = (short)((mag_y - offset_y) * scale_y);
  mag_z = (short)((mag_z - offset_z) * scale_z);


  if (err !=  AK09918_ERR_OK) {
    Serial.print("getRawData err: 0x");
    Serial.println(err, HEX);
  }
  //mag_x *= 15;  mag_y *= 15;  mag_z *= 15; // Now in .01 uT
  static unsigned long lastdbg = 0;

  if (millis() - lastdbg > 1000 ) {
    sprintf(fmtbuf, "a=[%d,%d,%d]; g=[%d,%d,%d]; m=[%d,%d,%d]", acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z);
    SerialUSB.println(fmtbuf);
    lastdbg = millis();
  }

  return err == AK09918_ERR_OK;
}
