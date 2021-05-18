// User interface

// Register a tap in the UI with the device oriented with given tilt
// Can also retrieve overall position (acc_*) from imu
extern void uitap(float tilt);

// Get current position to point arrow (0-360, 0=up)
extern int getuipos(void);

// Is UI active
extern bool uiactive(void);

