extern short acc_x, acc_y, acc_z;
extern short gyro_x, gyro_y, gyro_z;
extern short mag_x, mag_y, mag_z;

extern bool haveimu;


extern void imusetup();
extern void imuloop();
extern float getHeading(void);
extern void imucommand(const char *cmd);
