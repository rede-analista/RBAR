#include <Arduino.h>
#include "i2c_slave.h"
#include "servo_control.h"
#include "kinematics.h"
#include "trajectory.h"

void setup() {
    i2c_init();
    servo_init();
    trajectory_init();
}

void loop() {
    i2c_process();
    trajectory_update();
    servo_update();
}
