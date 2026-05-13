#pragma once
#include "globals.h"

void gait_init();
void gait_set_params(const GaitParams_t *params);
void gait_start();
void gait_stop();
void gait_update();
bool gait_running();

extern GaitParams_t gait_params;
