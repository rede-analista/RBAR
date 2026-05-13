#include <Arduino.h>
#include "i2c_slave.h"
#include "servo_control.h"
#include "kinematics.h"
#include "trajectory.h"
#include "gait.h"
#include "debug_serial.h"

void setup() {
    servo_init();
    trajectory_init();
    gait_init();
    i2c_init();
    debug_init();
}

void loop() {
    i2c_process();

    if (gait_running()) {
        gait_update();   // gait controla servo_set() diretamente
    } else {
        trajectory_update(); // interpolação ponto-a-ponto
    }

    servo_update();
    debug_update();
}
