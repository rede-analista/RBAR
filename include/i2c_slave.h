#pragma once
#include "globals.h"

void i2c_init();
void i2c_process();

extern volatile I2C_Command_t i2c_last_cmd;
extern volatile Vec3_t        i2c_target_pos;
extern volatile uint8_t       i2c_speed;
