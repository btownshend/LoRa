#include <AccelStepper.h>

class Needle  {
    const int STEPSPERREV = 720;
    const int NORTHOFFSET = -5;   // Offset in steps from sensor-based zero to position with needle pointing north,, positive values rotate needle CCWt

    AccelStepper stepper;
    int sensorVals[360];   // Sensor values for each angle
    int maxPos;  // Current angle with maximum sensor reading
    int spinning;  // Number of rotations of needle to make
    bool fieldtesting;  // True when testing field - disable stepping
    bool enabled;
 public:
    Needle(void);
    void loop(void);
    void setup(void);
    void command(const char *cmd);
    void stepperadvance(void); // Move ahead one step
    boolean run(void) { return stepper.run(); }
    int  timesinceactive(void);   // Return number of milliseconds since the stepper was last powered
    int lastenabled; // Time (msec) that stepper was last enabled
 private:
    void dumpsensor(void);
    int getsensor(void);
    void sensorcheck(void);
    void stepperfield(void);
    void gotoangle(float angle);
    void adjuststepper(void);
    void enable(void);
    void disable(void);
    void move(long relative);
    void moveTo(long absolute);
    void setuptimer(void);
    static void TimerHandler0(void);
};

extern Needle needle;

inline void needleloop() { needle.loop(); }
