#pragma once
#include "globals.h"

void trajectory_init();
void trajectory_set_target(const JointAngles_t *target, uint8_t speed);
void trajectory_update();
bool trajectory_done();

extern JointAngles_t traj_current;
