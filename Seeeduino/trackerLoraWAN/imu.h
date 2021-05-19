extern short acc_x, acc_y, acc_z;
extern short orient_x, orient_y,orient_z;  // Orientation, updated only when device is not accelerating (other than gravity)
extern short gyro_x, gyro_y, gyro_z;
extern short mag_x, mag_y, mag_z;

extern bool haveimu;


extern void imusetup(void);
extern void imuloop(void);
extern float getHeading(void);
extern void imucommand(const char *cmd);
extern bool isstill(void);
