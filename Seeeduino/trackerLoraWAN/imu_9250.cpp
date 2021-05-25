#include "globals.h"
#ifdef IMU_9250

#include <Scheduler.h>
#include <FlashStorage.h>
#include <Wire.h>
// If there's a complie error in I2Cdev.cpp, need to add #define BUFFER_LENGTH 32 in I2Cdev.h in library
#include "imu.h"
#include "ui.h"
#include "needle.h"
#include "imucal.h"

// 9DOF
float acc_mag;  // Total magnitude of acceleration
float acc_external;  // External acceleration
const float stillaccel = 0.01f;  // External acceleration less than this to be considered still

const float UTPERUNIT=0.15f;
const float GPERUNIT=16.0/32768;
const float DPSPERUNIT=2000.0/32768;


FlashStorage(calStorage, magCalType);
static magCalType magCal;
IMU imu;

void IMU::setup() {
    notice("imusetup\n");
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
	error("Unable to communicate with MPU-9250\n");
	return;
    }

    // Use setSensors to turn on or off MPU-9250 sensors.
    // Any of the following defines can be combined:
    // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
    // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    // Enable all sensors:
    if (imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS) != INV_SUCCESS) {
	error("Unable to setSensors()\n");
	return;
    }

    // Use setGyroFSR() and setAccelFSR() to configure the
    // gyroscope and accelerometer full scale ranges.
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    if (imu.setGyroFSR(2000) != INV_SUCCESS) { // Set gyro to 2000 dps
	error("Unable to setGyroFSR(2000)\n");
	return;
    }
    // Accel options are +/- 2, 4, 8, or 16 g
    if (imu.setAccelFSR(16)  != INV_SUCCESS) {  // Set accel to +/-16g
	error("Unable to setAccelFSR()\n");
	return;
    }
    // Note: the MPU-9250's magnetometer FSR is set at 
    // +/- 4912 uT (micro-tesla's)

    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    if (imu.setLPF(42) != INV_SUCCESS) { // Set LPF corner frequency to 5Hz
	error("Unable to setLPF()\n");
	return;
    }

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    if (imu.setSampleRate(100) != INV_SUCCESS) { // Set sample rate to 10Hz
	error("Unable to setSampleRate()\n");
	return;
    } 
    
    // Likewise, the compass (magnetometer) sample rate can be
    // set using the setCompassSampleRate() function.
    // This value can range between: 1-100Hz
    if (imu.setCompassSampleRate(100) != INV_SUCCESS) { // Set mag rate to 10Hz
	error("Unable to setSensors()\n");
	return;
    }

    if (imu.configureFifo(INV_XYZ_GYRO || INV_XYZ_ACCEL) != INV_SUCCESS) {
	error("Unable to configureFifo()\n");
	return;
    }
    
    // Enable tap detection in the DMP. Set FIFO sample rate to 10Hz.
    if (imu.dmpBegin(DMP_FEATURE_TAP|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_RAW_GYRO|DMP_FEATURE_6X_LP_QUAT , 100) != INV_SUCCESS) {
	error("Unable to dmpBegin()\n");
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
	error("Unable to dmpSetTap()\n");
	return;
    }
  
    notice("9DOF Initialized: sampling Rate: AG:%d, M: %d\n",(int)imu.getSampleRate(), (int)imu.getCompassSampleRate());

    magCal = calStorage.read();
    if (magCal.offset[0]==0.0) {
	warning("No magnetic calibration available - defaulting to self-calibration\n");
	magCal=defaultMagCal;
	warning("Initialized magCal from defaults\n");
    }
    haveimu=true;

    // Initialize AHRS filter at 100Hz update rate
    filter.begin(100);

    // Verify constants
    if (DPSPERUNIT != 1.0/imu.getGyroSens())
	warning("DPSPERUNIT=%f, but gyro sensitivity=%f\n", DPSPERUNIT, 1.0/imu.getGyroSens());
    if (GPERUNIT!=1.0/imu.getAccelSens())
	warning("GPERUNIT=%f, but acc sensitivity=%f\n", GPERUNIT, 1.0/imu.getAccelSens());
    if (UTPERUNIT!=imu.getMagSens())
	warning("UTPERUNIT=%f, but mag sensitivity=%f\n", UTPERUNIT, imu.getMagSens());
}

void IMU::updateCalibration(void) {
    if (magCal.offset[0]==0.0) {
	static short mag_xmin = -10, mag_xmax = 10, mag_ymin = -10, mag_ymax = 10, mag_zmin = -10, mag_zmax = 10;
	bool changed = false;
	if (imu.mx < mag_xmin) {
	    mag_xmin = imu.mx;
	    changed = true;
	}
	if (imu.mx > mag_xmax) {
	    mag_xmax = imu.mx;
	    changed = true;
	}
	if (imu.my < mag_ymin) {
	    mag_ymin = imu.my;
	    changed = true;
	}
	if (imu.my > mag_ymax) {
	    mag_ymax = imu.my;
	    changed = true;
	}
	if (imu.mz < mag_zmin) {
	    mag_zmin = imu.mz;
	    changed = true;
	}
	if (imu.mz > mag_zmax) {
	    mag_zmax = imu.mz;
	    changed = true;
	}
	static float scale_x = 1, scale_y = 1, scale_z = 1;
	static int offset_x = 0, offset_y = 0, offset_z = 0;
	if (changed) {
	    notice("New mag range: [%d,%d]; [%d,%d]; [%d,%d]\n", mag_xmin, mag_xmax, mag_ymin, mag_ymax, mag_zmin, mag_zmax);
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
	    notice("Offset=%d,%d,%d Scale=%.2f,%.2f,%.2f\n", offset_x, offset_y, offset_z, scale_x, scale_y, scale_z);
	}
	// internal calibration
	mag_x = (short)((imu.mx - offset_x) * scale_x);
	mag_y = (short)((imu.my - offset_y) * scale_y);
	mag_z = (short)((imu.mz - offset_z) * scale_z);
    } else {
	// stored calibration
	float c[3];
	c[0]=imu.mx-magCal.offset[0];
	c[1]=imu.my-magCal.offset[1];
	c[2]=imu.mz-magCal.offset[2];
	mag_x = (short)(c[0]*magCal.mat[0][0]+c[1]*magCal.mat[1][0]+c[2]*magCal.mat[2][0]);
	mag_y = (short)(c[0]*magCal.mat[0][1]+c[1]*magCal.mat[1][1]+c[2]*magCal.mat[2][1]);
	mag_z = (short)(c[0]*magCal.mat[0][2]+c[1]*magCal.mat[1][2]+c[2]*magCal.mat[2][2]);
	static unsigned long lastdebug=0;
	if (millis()-lastdebug>5000) {
	    notice("raw=[%d,%d,%d], calib=[%d,%d,%d]\n",imu.mx,imu.my,imu.mz,mag_x,mag_y,mag_z);
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
    // Check for new data in the FIFO
    static int nsamps=0;
    static int nyield=0;
    
    while ( imu.fifoAvailable() )  {
	// Updates acc, gyro values from FIFO
	nsamps++;
	imu.dmpUpdateFifo();

	// Got new data, save it in NWU orientation
	gyro_x = imu.gx; gyro_y = imu.gy; gyro_z =-imu.gz;  // Normal orientation - swapped from magnet
	acc_x = imu.ax; acc_y = imu.ay; acc_z = imu.az;  // Normal orientation - swapped from magnet

#ifndef IMUTEST
	if (needle.timesinceactive() > 200) {
#endif
	    // Only update the magnetometer when the stepping motor has been off 
	    imu.update(UPDATE_COMPASS);
	    rawmag_x = imu.mx; rawmag_y = imu.my; rawmag_z = imu.mz;  // Magnetometer swaps x & y relative to accel/gyro and inverts z
	    //rawmag_x = imu.mx; rawmag_y = imu.my; rawmag_z = imu.mz;  // Normal orientation
	    updateCalibration();  
#ifdef LAYFLAT
	    filter.update(gyro_x*DPSPERUNIT, gyro_y*DPSPERUNIT,gyro_z*DPSPERUNIT,acc_x*GPERUNIT, acc_y*GPERUNIT, acc_z*GPERUNIT, mag_x*UTPERUNIT, mag_y*UTPERUNIT, mag_z*UTPERUNIT);
#else
	    // On side
	    filter.update(gyro_y*DPSPERUNIT, gyro_z*DPSPERUNIT,gyro_x*DPSPERUNIT,acc_y*GPERUNIT, acc_z*GPERUNIT, acc_x*GPERUNIT, mag_x*UTPERUNIT, -mag_z*UTPERUNIT, mag_y*UTPERUNIT);
#endif
#ifndef IMUTEST
	} else
	    // Skip compass reading, free-run the filter
	    filter.updateIMU(gyro_x*DPSPERUNIT, gyro_y*DPSPERUNIT,gyro_z*DPSPERUNIT,acc_x*GPERUNIT, acc_y*GPERUNIT, acc_z*GPERUNIT);
#endif

	acc_mag = sqrt(1.0f*acc_x*acc_x+1.0f*acc_y*acc_y+1.0f*acc_z*acc_z)*GPERUNIT;
	static float meanmag=1.0f;
	meanmag=meanmag*0.999+acc_mag*.001;
	acc_external = fabs(acc_mag-meanmag);

	if (isstill()) {
	    orient_x=acc_x;orient_y=acc_y;orient_z=acc_z;
	}
    
	static unsigned long lastdbg = 0;
#ifdef IMUTEST
	if (millis() - lastdbg > 1000 ) {
#else
	if (millis() - lastdbg > 10000 ) {
#endif
	    double field = sqrt(1.0 * mag_x * mag_x + 1.0 * mag_y * mag_y + 1.0 * mag_z * mag_z);
	    notice("   a=[%d,%d,%d] (mean:%.2f); g=[%d,%d,%d]; m=[%d,%d,%d]=%.0f, raw=[%d,%d,%d]\n", acc_x, acc_y, acc_z, meanmag, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z,field,rawmag_x,rawmag_y,rawmag_z);
	    imu.computeEulerAngles(true);
	    notice("Raw:    Roll: %4.0f, Pitch: %4.0f, Yaw: %4.0f, Heading: %.0f, Tilt: %.0f\n", imu.roll,imu.pitch,imu.yaw,getHeading(),gettilt());
	    notice("Filter: Roll: %4.0f, Pitch: %4.0f, Yaw: %4.0f\n", filter.getRoll(), filter.getPitch(), filter.getYaw());
	    notice("Rate: %.0f  Y:%d,S:%d\n", nsamps*1000.0f/(millis()-lastdbg),nyield,nsamps);
	    nsamps=0;
	    nyield=0;
	    lastdbg = millis();
	}

	// Check for new tap data by polling tapAvailable
	if ( imu.tapAvailable() )   {
	    notice("Tap %d, %d\n",imu.getTapDir(), imu.getTapCount());
	    uitap(gettilt());
	}
    }

    // Check stack
    static int minstack=100000; minstack=stackcheck("IMU",minstack);
    yield();
    nyield++;
}

void IMU::monitor(bool matlabMode) {
    // Go into IMU monitoring until another character is received
    // Do not call yield or delay or another task will execute
    if (matlabMode)
	SerialUSB.println("+++IMAT+++");
    else
	SerialUSB.println("+++IMON+++");
	
    while (SerialUSB.available())
	SerialUSB.read();
    int spos=0;
    int cntr=0;
    while (true) {
	if (matlabMode && cntr>=100) {
	    sprintf(fmtbuf,"MAGSET %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
		    magCal.offset[0],magCal.offset[1],magCal.offset[2],
		    magCal.mat[0][0],magCal.mat[0][1],magCal.mat[0][2],
		    magCal.mat[1][0],magCal.mat[1][1],magCal.mat[1][2],
		    magCal.mat[2][0],magCal.mat[2][1],magCal.mat[2][2]);
	    SerialUSB.println(fmtbuf);
	    cntr=0;
	}
	cntr++;

	if ( imu.fifoAvailable() )  {
	    // Updates acc, gyro values from FIFO
	    imu.dmpUpdateFifo();
	    // On-demand read of compass
	    imu.update(UPDATE_COMPASS);
	    if (matlabMode)
		sprintf(fmtbuf,"Mat:%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",millis(), imu.ax,imu.ay,imu.az,imu.gx,imu.gy,imu.gz,imu.mx,imu.my,imu.mz,spos);
	    else // Compat with MotionCal
		sprintf(fmtbuf,"Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d", imu.ax,imu.ay,imu.az,imu.gx,imu.gy,imu.gz,imu.mx,imu.my,imu.mz);
	    SerialUSB.println(fmtbuf);
	}
	if (SerialUSB.available()) {
	    int c=SerialUSB.read();
	    if (c=='x')
		break;
	    else if (c=='s') {
		needle.stepperadvance();
		spos++;
	    } else {
		notice("Got: %c\n",c);
	    }
	}
    }
    SerialUSB.println("+++END+++");
}

void IMU::command(const char *cmd) {
    if (strcmp(cmd,"MON")==0)
	monitor(false);
    else if (strcmp(cmd,"MAT")==0) {
	monitor(true);
    } else if (strncmp(cmd,"MAGSET",6)==0) {
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
	warning("Expected IMON,  IMAGSET, or ICAL\n");
}
#endif /* IMU_9250 */
