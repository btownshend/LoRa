#include <SparkFunMPU9250-DMP.h>
#include <MadgwickAHRS.h>
//#include <NXPMotionSense.h>
//#include <Adafruit_AHRS.h>

class IMU {
    MPU9250_DMP imu;
    Madgwick filter;
    //    Adafruit_NXPSensorFusion filter;
    float fgx,fgy,fgz,fax,fay,faz,fmx,fmy,fmz;  // Current readings in natural units (Deg/s, g, uT), with axes appropriately adjusted
 public:
    short mag_x, mag_y, mag_z;
    bool haveimu;

    void setup(void);
    void loop(void);
    float getHeading(void);
    void command(const char *cmd);
    bool isstill(void);
    short getRawMag(int axis) const;
    float getUpRotation(void);
    float getTilt(void);
 private:
    void monitor(bool);
    void updateCalibration(void);
};

extern IMU imu;

inline void imuloop(void) { imu.loop(); }

//#define IMUTEST  // Disable all other threads

