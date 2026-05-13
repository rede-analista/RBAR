#include <Arduino.h>
#include "i2c_slave.h"
#include "servo_control.h"
#include "kinematics.h"
#include "trajectory.h"
#include "debug_serial.h"

void setup() {
    servo_init();
    trajectory_init();
    i2c_init();
    debug_init();
}

void loop() {
    i2c_process();
    trajectory_update();
    servo_update();
    debug_update();
}
