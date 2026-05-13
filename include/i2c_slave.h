#pragma once
#include "globals.h"

void i2c_init();
void i2c_process();

// Apenas rx_ready/rx_buf precisam de volatile (escritos na ISR).
// Os campos abaixo são escritos somente no loop principal.
extern I2C_Command_t i2c_last_cmd;
extern Vec3_t        i2c_target_pos;
extern uint8_t       i2c_speed;
