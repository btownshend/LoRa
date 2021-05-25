#include <SparkFunMPU9250-DMP.h>
#include <MadgwickAHRS.h>
//#include <NXPMotionSense.h>
//#include <Adafruit_AHRS.h>

class IMU {
    MPU9250_DMP imu;
    Madgwick filter;
    //    Adafruit_NXPSensorFusion filter;
 public:
    short orient_x, orient_y,orient_z;  // Orientation, updated only when device is not accelerating (other than gravity)
    short mag_x, mag_y, mag_z;
    bool haveimu;

    void setup(void);
    void loop(void);
    float getHeading(void);
    void command(const char *cmd);
    bool isstill(void);
 private:
    float gettilt(void);
    void monitor(bool);
    void updateCalibration(void);
};

extern IMU imu;

inline void imuloop(void) { imu.loop(); }

#define IMUTEST  // Disable all other threads

