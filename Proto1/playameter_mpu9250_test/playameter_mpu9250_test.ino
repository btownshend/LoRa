#include "MPU9250.h"

MPU9250 mpu;

void setup() {
    SerialUSB.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            SerialUSB.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
}

void print_roll_pitch_yaw() {
    SerialUSB.print("Yaw, Pitch, Roll: ");
    SerialUSB.print(mpu.getYaw(), 2);
    SerialUSB.print(", ");
    SerialUSB.print(mpu.getPitch(), 2);
    SerialUSB.print(", ");
    SerialUSB.println(mpu.getRoll(), 2);
}
