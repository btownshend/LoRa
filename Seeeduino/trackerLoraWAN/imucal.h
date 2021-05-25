#define SEEEDUINO3

typedef struct {
    // Calibration of magnetometer
    // Calibrated val = (raw-offset)*mat
    float offset[3];
    float mat[3][3];   // calibration matrix,  mat[ij][j] is row i, col j
} magCalType;

#ifdef SEEEDUINO4
// Calibration using MotionCal 5/25/21
const magCalType defaultMagCal = {
    {16.79/.15,-.28/.15,0.92/.15},
    {{1.062,.023,-.021},   // By row, then col
     {.023,.981,.027},
     {-.021,.027,.962}}};  //  field=55.43;
#endif

#ifdef SEEEDUINO3
// Calibration using Matlab 5/25/21
const magCalType defaultMagCal =
{{96.76,155.12,-195.22},
{{1.00,0.00,0.00},
{0.00,1.00,0.00},
{0.00,0.00,1.00}}}; // Total=225.62
#endif
