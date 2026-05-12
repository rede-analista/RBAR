#pragma once
#include "globals.h"

void servo_init();
void servo_set(const JointAngles_t *angles);
void servo_update();
