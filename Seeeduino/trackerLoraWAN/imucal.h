#define SEEEDUINO4

typedef struct {
    // Calibration of magnetometer
    // Calibrated val = (raw-offset)*mat
    float offset[3];
    float mat[3][3];   // calibration matrix,  mat[ij][j] is row i, col j
} magCalType;

#ifdef SEEEDUINO4
// Calibration using MotionCal 5/25/21
const magCalType defaultMagCal =
{{160.26,-12.42,7.11},
{{1.00,0.00,0.00},
{0.00,1.00,0.00},
{0.00,0.00,1.00}}}; // Total=266.66
#endif

#ifdef SEEEDUINO3
// Calibration using Matlab 5/25/21
const magCalType defaultMagCal =
{{96.76,155.12,-195.22},
{{1.00,0.00,0.00},
{0.00,1.00,0.00},
{0.00,0.00,1.00}}}; // Total=225.62
#endif
