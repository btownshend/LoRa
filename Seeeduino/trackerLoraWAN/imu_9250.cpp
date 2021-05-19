#include "globals.h"
#ifdef IMU_9250

#include <Scheduler.h>
#include <FlashStorage.h>
#include <Wire.h>
// If there's a complie error in I2Cdev.cpp, need to add #define BUFFER_LENGTH 32 in I2Cdev.h in library
#include "imu.h"
#include "ui.h"
#include "stepper.h"

// 9DOF
float acc_mag;  // Total magnitude of acceleration
float acc_external;  // External acceleration
const float stillaccel = 0.01f;  // External acceleration less than this to be considered still


typedef struct {
    // Calibration of magnetometer
    // Calibrated val = (raw-offset)*mat
    float offset[3];
    float mat[3][3];   // calibration matrix,  mat[ij][j] is row i, col j
} magCalType;

FlashStorage(calStorage, magCalType);
static magCalType magCal;
IMU imu;

void IMU::setup() {
    SerialUSB.println("imusetup");
    delay(100);
    
    // Enable Grove connectors
    digitalWrite(38, HIGH);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // Call imu.begin() to verify communication with and
    // initialize the MPU-9250 to it's default values.
    // Most functions return an error code - INV_SUCCESS (0)
    // indicates the IMU was present and successfully set up
    if (imu.begin() != INV_SUCCESS) {
	SerialUSB.println("Unable to communicate with MPU-9250");
	return;
    }

    // Use setSensors to turn on or off MPU-9250 sensors.
    // Any of the following defines can be combined:
    // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
    // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    // Enable all sensors:
    if (imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS) != INV_SUCCESS) {
	SerialUSB.println("Unable to setSensors()");
	return;
    }

    // Use setGyroFSR() and setAccelFSR() to configure the
    // gyroscope and accelerometer full scale ranges.
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    if (imu.setGyroFSR(2000) != INV_SUCCESS) { // Set gyro to 2000 dps
	SerialUSB.println("Unable to setGyroFSR(2000)");
	return;
    }
    // Accel options are +/- 2, 4, 8, or 16 g
    if (imu.setAccelFSR(16)  != INV_SUCCESS) {  // Set accel to +/-2g
	SerialUSB.println("Unable to setAccelFSR()");
	return;
    }
    // Note: the MPU-9250's magnetometer FSR is set at 
    // +/- 4912 uT (micro-tesla's)

    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    if (imu.setLPF(5) != INV_SUCCESS) { // Set LPF corner frequency to 5Hz
	SerialUSB.println("Unable to setLPF()");
	return;
    }

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    if (imu.setSampleRate(10) != INV_SUCCESS) { // Set sample rate to 10Hz
	SerialUSB.println("Unable to setSampleRate()");
	return;
    } 
    
    // Likewise, the compass (magnetometer) sample rate can be
    // set using the setCompassSampleRate() function.
    // This value can range between: 1-100Hz
    if (imu.setCompassSampleRate(10) != INV_SUCCESS) { // Set mag rate to 10Hz
	SerialUSB.println("Unable to setSensors()");
	return;
    }

    // Enable tap detection in the DMP. Set FIFO sample rate to 10Hz.
    if (imu.dmpBegin(DMP_FEATURE_TAP|DMP_FEATURE_6X_LP_QUAT , 10) != INV_SUCCESS) {
	SerialUSB.println("Unable to dmpBegin()");
	return;
    }
    // dmpSetTap parameters, in order, are:
    // x threshold: 1-1600 (0 to disable)
    // y threshold: 1-1600 (0 to disable)
    // z threshold: 1-1600 (0 to disable)
    // (Threshold units are mg/ms)
    // taps: Minimum number of taps needed for interrupt (1-4)
    // tap time: milliseconds between valid taps
    // tap time multi: max milliseconds between multi-taps
    unsigned short xThresh = 100;   // Disable x-axis tap
    unsigned short yThresh = 100;   // Disable y-axis tap
    unsigned short zThresh = 100; // Set z-axis tap thresh to 100 mg/ms
    unsigned char taps = 1;       // Set minimum taps to 1
    unsigned short tapTime = 100; // Set tap time to 100ms
    unsigned short tapMulti = 1000;// Set multi-tap time to 1s
    if (imu.dmpSetTap(xThresh, yThresh, zThresh, taps, tapTime, tapMulti) != INV_SUCCESS) {
	SerialUSB.println("Unable to dmpSetTap()");
	return;
    }
  
    SerialUSB.println("9DOF Initialized");
    sprintf(fmtbuf,"Sampling Rate: AG:%d, M: %d",(int)imu.getSampleRate(), (int)imu.getCompassSampleRate());
    SerialUSB.println(fmtbuf);

    magCal = calStorage.read();
    if (magCal.offset[0]==0.0) {
	SerialUSB.println("No magnetic calibration available - defaulting to self-calibration");
    }
    haveimu=true;
}

void IMU::updateCalibration(void) {
    static short mag_xmin = -10, mag_xmax = 10, mag_ymin = -10, mag_ymax = 10, mag_zmin = -10, mag_zmax = 10;
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
    if (magCal.offset[0]==0.0) {
	// internal calibration
	mag_x = (short)((rawmag_x - offset_x) * scale_x);
	mag_y = (short)((rawmag_y - offset_y) * scale_y);
	mag_z = (short)((rawmag_z - offset_z) * scale_z);
    } else {
	// stored calibration
	float c[3];
	c[0]=rawmag_x-magCal.offset[0];
	c[1]=rawmag_y-magCal.offset[1];
	c[2]=rawmag_z-magCal.offset[2];
	mag_x = (short)(c[0]*magCal.mat[0][0]+c[1]*magCal.mat[1][0]+c[2]*magCal.mat[2][0]);
	mag_y = (short)(c[0]*magCal.mat[0][1]+c[1]*magCal.mat[1][1]+c[2]*magCal.mat[2][1]);
	mag_z = (short)(c[0]*magCal.mat[0][2]+c[1]*magCal.mat[1][2]+c[2]*magCal.mat[2][2]);
	static unsigned long lastdebug=0;
	if (millis()-lastdebug>5000) {
	    sprintf(fmtbuf,"raw=[%d,%d,%d], calib=[%d,%d,%d]",rawmag_x,rawmag_y,rawmag_z,mag_x,mag_y,mag_z);
	    SerialUSB.println(fmtbuf);
	    lastdebug=millis();
	}
    }
}


float IMU::getHeading(void) {
    // Get current heading in degrees

    // roll/pitch in radian
    double roll = atan2((float)acc_y, (float)acc_z);
    double pitch = atan2(-(float)acc_x, sqrt((float)acc_y * acc_y + (float)acc_z * acc_z));

    double Xheading = mag_x * cos(pitch) + mag_y * sin(roll) * sin(pitch) + mag_z * cos(roll) * sin(pitch);
    double Yheading = mag_y * cos(roll) - mag_z * sin(pitch);

    const double declination = -6;   // Magnetic declination
    float heading = 180 + 57.3 * atan2(Yheading, Xheading) + declination;
    
    return heading;
}

bool IMU::isstill(void) {
    return acc_external < stillaccel;
}

float IMU::gettilt(void) {
    float xymag=sqrt(1.0f*orient_x*orient_x+1.0f*orient_y*orient_y);
    return atan2(xymag,1.0f*orient_z)*57.3;
}

void IMU::loop() {
    // Check for new data
    if (imu.dataReady()) {
	// Got new data, save it
	imu.update(UPDATE_ACCEL|UPDATE_GYRO|UPDATE_COMPASS);
	rawmag_x = imu.mx; rawmag_y = -imu.mz; rawmag_z = imu.my;  // Module is mounted on side (differently from gyro and acc)
	//rawmag_x = imu.mx; rawmag_y = imu.my; rawmag_z = imu.mz;  // Normal orientation
	gyro_x = imu.gy; gyro_y = imu.gz; gyro_z = imu.gx;  // Mounted on side
	//gyro_x = imu.gx; gyro_y = imu.gy; gyro_z = imu.gz;  // Normal orientation
	acc_x = imu.ay; acc_y = imu.az; acc_z = imu.ax;  // Mounted on side
	//acc_x = imu.ax; acc_y = imu.ay; acc_z = imu.az;  // Normal orientation
	updateCalibration();  

	acc_mag = sqrt(1.0f*acc_x*acc_x+1.0f*acc_y*acc_y+1.0f*acc_z*acc_z)/2048;
	static float meanmag=1.0f;
	meanmag=meanmag*0.999+acc_mag*.001;
	acc_external = fabs(acc_mag-meanmag);

	if (isstill()) {
	    orient_x=acc_x;orient_y=acc_y;orient_z=acc_z;
	}
    
	static unsigned long lastdbg = 0;
	if (millis() - lastdbg > 10000 ) {
	    double field = sqrt(1.0 * mag_x * mag_x + 1.0 * mag_y * mag_y + 1.0 * mag_z * mag_z);
	    sprintf(fmtbuf, "a=[%d,%d,%d] (mean:%.2f); g=[%d,%d,%d]; m=[%d,%d,%d]=%.0f, raw=[%d,%d,%d]", acc_x, acc_y, acc_z, meanmag, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z,field,rawmag_x,rawmag_y,rawmag_z);
	    SerialUSB.print(fmtbuf);
	    imu.computeEulerAngles(true);
	    sprintf(fmtbuf, ", Roll: %.0f, Pitch: %.0f, Yaw: %.0f, Heading: %.0f, Tilt: %.0f", imu.roll,imu.pitch,imu.yaw,getHeading(),gettilt());
	    SerialUSB.println(fmtbuf);
	    lastdbg = millis();
	}
    }

    // Check for new data in the FIFO
    while ( imu.fifoAvailable() )  {
	// DMP FIFO must be updated in order to update tap data
	imu.dmpUpdateFifo();
	// Check for new tap data by polling tapAvailable
	if ( imu.tapAvailable() )   {
	    sprintf(fmtbuf,"Tap %d, %d\n",imu.getTapDir(), imu.getTapCount());
	    SerialUSB.println(fmtbuf);
	    uitap(gettilt());
	}
    }

    // Check stack
    static int minstack=100000; minstack=stackcheck("IMU",minstack);
    yield();
}

void IMU::monitor(void) {
    // Go into IMU monitoring until another character is received
    // Do not call yield or delay or another task will execute
    SerialUSB.println("+++IMON+++");
    while (SerialUSB.available())
	SerialUSB.read();
    int spos=0;
    while (true) {
	if (imu.dataReady()) {
	    imu.update(UPDATE_ACCEL|UPDATE_GYRO|UPDATE_COMPASS);
	    sprintf(fmtbuf,"Raw:%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",millis(), imu.ax,imu.ay,imu.az,imu.gx,imu.gy,imu.gz,imu.mx,imu.my,imu.mz,spos);
	    SerialUSB.println(fmtbuf);
	}
	if (SerialUSB.available()) {
	    int c=SerialUSB.read();
	    SerialUSB.print("Got: ");
	    SerialUSB.println(c);
	    if (c=='x')
		break;
	    else if (c=='s') {
		stepperadvance();
		spos++;
	    }
	}
    }
    SerialUSB.println("+++END+++");
}

void IMU::command(const char *cmd) {
    if (strcmp(cmd,"MON")==0)
	monitor();
    else if (strncmp(cmd,"MAGSET",6)==0) {
	float offset[3], mat[3][3];
	int nr=sscanf(cmd,"MAGSET %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
		      &magCal.offset[0],&magCal.offset[1],&magCal.offset[2],
		      &magCal.mat[0][0],&magCal.mat[0][1],&magCal.mat[0][2],
		      &magCal.mat[1][0],&magCal.mat[1][1],&magCal.mat[1][2],
		      &magCal.mat[2][0],&magCal.mat[2][1],&magCal.mat[2][2]);
	if (nr==12) {
	    calStorage.write(magCal);
	    SerialUSB.println("MAGSET ok");
	} else {
	    SerialUSB.print("MAGSET only parsed ");
	    SerialUSB.print(nr);
	    SerialUSB.println("/12 fields");
	}
    } else if (strncmp(cmd,"CAL",3)==0) {
	//disableStepper();
	
	//enableStepper();
    } else
	SerialUSB.println("Expected IMON,  IMAGSET, or ICAL");
}
#endif /* IMU_9250 */
