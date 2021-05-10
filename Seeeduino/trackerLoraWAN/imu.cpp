#include <Scheduler.h>
#include "AK09918.h"
#include "ICM20600.h"
#include <Wire.h>
// If there's a complie error in I2Cdev.cpp, need to add #define BUFFER_LENGTH 32 in I2Cdev.h in library

#include "globals.h"
#include "imu.h"
#include "ui.h"

// 9DOF
short acc_x, acc_y, acc_z;
short gyro_x, gyro_y, gyro_z;
short rawmag_x, rawmag_y, rawmag_z;  // As read from AK09918
short mag_x, mag_y, mag_z;  // After low pass filtering and scaling
bool haveimu = false;
float mag_offset[3];
float mag_mat[3][3];

// Private
static AK09918 ak09918;
static ICM20600 icm20600(true);


void imusetup() {
    SerialUSB.println("imusetup");

    // Enable Grove connectors
    digitalWrite(38, HIGH);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    AK09918_err_type_t err = ak09918.initialize();
    if (err != AK09918_ERR_OK) {
      Serial.print("Error initializing AK09918: 0x");
      Serial.println(err, HEX);
      return;
    }

    delay(100);
    err = ak09918.selfTest();
    if (err != AK09918_ERR_OK) {
      Serial.print("Error in selfTest of AK09918: 0x");
      Serial.println(err, HEX);
      return;
    }

    icm20600.initialize();
    ak09918.switchMode(AK09918_POWER_DOWN);
    ak09918.switchMode(AK09918_CONTINUOUS_100HZ);
    delay(100);
    SerialUSB.println("9DOF Initialized");
    haveimu=true;
  }

bool updateMag(void) {
    AK09918_err_type_t err = ak09918.isDataReady();
    if (err==AK09918_ERR_NOT_RDY)
	return false;
    else if (err != AK09918_ERR_OK ) {
	SerialUSB.print("AK09918.isDataReady() -> 0x");
	SerialUSB.println(err,HEX);
    }
    int32_t x, y, z;
    err = ak09918.getRawData(&x, &y, &z);  // raw data is in units of 0.15uT
    if (err !=  AK09918_ERR_OK) {
	Serial.print("getRawData err: 0x");
	Serial.println(err, HEX);
	return false;
    }
    //rawmag_x = x; rawmag_y = y; rawmag_z = z;
    rawmag_x = x; rawmag_y = z; rawmag_z = -y;  // Module is mounted on side

    static short mag_xmin = -100, mag_xmax = 100, mag_ymin = -100, mag_ymax = 100, mag_zmin = -100, mag_zmax = 100;
    bool changed = false;
    if (rawmag_x < mag_xmin) {
	mag_xmin = rawmag_x;
	changed = true;
    }
    if (rawmag_x > mag_xmax) {
	mag_xmax = rawmag_x;
	changed = true;
    }
    if (rawmag_y < mag_ymin) {
	mag_ymin = rawmag_y;
	changed = true;
    }
    if (rawmag_y > mag_ymax) {
	mag_ymax = rawmag_y;
	changed = true;
    }
    if (rawmag_z < mag_zmin) {
	mag_zmin = rawmag_z;
	changed = true;
    }
    if (rawmag_z > mag_zmax) {
	mag_zmax = rawmag_z;
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
    rawmag_x = (short)((rawmag_x - offset_x) * scale_x);
    rawmag_y = (short)((rawmag_y - offset_y) * scale_y);
    rawmag_z = (short)((rawmag_z - offset_z) * scale_z);

    // Average to form mag_*
    mag_x-=(mag_x - rawmag_x*100)/100;
    mag_y-=(mag_y-  rawmag_y*100)/100;
    mag_z-=(mag_z- rawmag_z*100)/100;
    //mag_x *= 15;  mag_y *= 15;  mag_z *= 15; // Now in .01 uT
    return true;
}
	
void updateAccGyro() {
    // Module mounted on side, so x=mz, y=mx, z=-my;
    acc_x = icm20600.getRawAccelerationX();
    acc_z = -icm20600.getRawAccelerationY();
    acc_y = icm20600.getRawAccelerationZ();
    gyro_x = icm20600.getRawGyroscopeX();
    gyro_z = -icm20600.getRawGyroscopeY();
    gyro_y = icm20600.getRawGyroscopeZ();
}

void detectGestures() {
    // Detect taps on the size
    static unsigned long lasttap=0;  // Time in ms of last tap
    static int nstill = 0;   // Number of samples of no external acceleration
    static float tilt = 0;  // Tilt in degrees (0=lying flat)
    const float tapaccel = 0.5f;  // Minimum tap acceleration
    const float stillaccel = 0.1f;  // External acceleration less than this to be considered still
    const float minstill = 5;   // Number of samples that must be still before tap
    const int holdoff = 100;  // Hold-off in msec before another tap is recognized
    float acc_mag = sqrt(1.0f*acc_x*acc_x+1.0f*acc_y*acc_y+1.0f*acc_z*acc_z)/2048;
    float external = fabs(acc_mag-1);
    if (external>=tapaccel && nstill>=minstill && millis()-lasttap> holdoff) {
	// New tap
	sprintf(fmtbuf,"TAP: acc=%.1f, nstill=%d, tilt=%.0f",external, nstill, tilt);
	SerialUSB.println(fmtbuf);
	uitap(tilt);
    }

    if (external<stillaccel) {
	float xymag=sqrt(1.0f*acc_x*acc_x+1.0f*acc_y*acc_y);
	tilt= atan2(xymag,1.0f*acc_z)*57.3;
	nstill++;
    } else
	nstill=0;
}

bool update9DOF() {
    updateAccGyro();
    if (updateMag()) {
	// The mag update also sets the update rate in general
	detectGestures();
	return true;
    }
    return false;
}

float getHeading(void) {
    // Get current heading in degrees

    // roll/pitch in radian
    double roll = atan2((float)acc_y, (float)acc_z);
    double pitch = atan2(-(float)acc_x, sqrt((float)acc_y * acc_y + (float)acc_z * acc_z));

    double Xheading = mag_x * cos(pitch) + mag_y * sin(roll) * sin(pitch) + mag_z * cos(roll) * sin(pitch);
    double Yheading = mag_y * cos(roll) - mag_z * sin(pitch);

    const double declination = -6;   // Magnetic declination
    float heading = 180 + 57.3 * atan2(Yheading, Xheading) + declination;
    
    static unsigned long lastdbg = 0;

    if (millis() - lastdbg > 1000 ) {
	sprintf(fmtbuf, "Roll: %.0f, Pitch: %.0f, Heading: %.0f", (roll * 57.3),( pitch * 57.3),heading);
	SerialUSB.println(fmtbuf);
	lastdbg = millis();
    }

    return heading;
}

void imuloop() {
    update9DOF();
    static unsigned long lastdbg = 0;

    if (millis() - lastdbg > 1000 ) {
	double field = sqrt(1.0 * mag_x * mag_x + 1.0 * mag_y * mag_y + 1.0 * mag_z * mag_z);
	sprintf(fmtbuf, "a=[%d,%d,%d]; g=[%d,%d,%d]; m=[%d,%d,%d]=%.0f", acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z,field);
	SerialUSB.println(fmtbuf);
	lastdbg = millis();
    }

    //    delay(10);  // At most 100Hz update rate (calls yield)
  // Check stack
  static int minstack=100000; minstack=stackcheck("IMU",minstack);
  yield();
}

}
