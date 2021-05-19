#include <AccelStepper.h>

class Needle  {
    AccelStepper stepper;
    int sensorVals[360];   // Sensor values for each angle
    int maxPos;  // Current angle with maximum sensor reading
    const int STEPSPERREV = 720;
    const int NORTHOFFSET = -5;   // Offset in steps from sensor-based zero to position with needle pointing north,, positive values rotate needle CCWt
    int spinning = 0;  // Number of rotations of needle to make
    bool fieldtesting = false;  // True when testing field - disable stepping
 public:
    Needle(void);
    void loop(void);
    void setup(void);
    void command(const char *cmd);
    void stepperadvance(void); // Move ahead one step
    void run(void) { stepper.run(); }
 private:
    void dumpsensor(void);
    void sensorcheck(void);
    void stepperfield(void);
    void gotoangle(float angle);
    void adjuststepper(void);
};

extern Needle needle;

inline void needleloop() { needle.loop(); }
