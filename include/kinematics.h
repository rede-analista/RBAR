#pragma once
#include "globals.h"
#include <stdbool.h>

bool ik_solve(const Vec3_t *target, JointAngles_t *out);
void fk_solve(const JointAngles_t *angles, Vec3_t *out);
void ik_clamp(JointAngles_t *angles);
