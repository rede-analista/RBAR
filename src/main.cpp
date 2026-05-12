#include <Arduino.h>
#include "i2c_slave.h"
#include "servo_control.h"
#include "kinematics.h"
#include "trajectory.h"

void setup() {
    servo_init();
    trajectory_init();
    i2c_init();
}

void loop() {
    i2c_process();      // despacha comandos recebidos via I2C
    trajectory_update(); // avança interpolação → chama servo_set()
    servo_update();      // escreve posição atual nos servos
}
