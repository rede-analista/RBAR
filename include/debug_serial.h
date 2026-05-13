#pragma once
#include "globals.h"

#ifdef DEBUG_SERIAL

void debug_init();
void debug_update();
void debug_ik_result(const Vec3_t *target, const JointAngles_t *angles, bool ok);
void debug_fk_verify(const JointAngles_t *angles);

#else

inline void debug_init()   {}
inline void debug_update() {}
inline void debug_ik_result(const Vec3_t*, const JointAngles_t*, bool) {}
inline void debug_fk_verify(const JointAngles_t*) {}

#endif
