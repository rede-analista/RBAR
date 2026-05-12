#include "trajectory.h"
#include "servo_control.h"
#include <Arduino.h>

JointAngles_t traj_current = {0.0f, 0.0f, 90.0f};

static JointAngles_t traj_start;
static JointAngles_t traj_end;
static uint32_t      traj_start_ms   = 0;
static uint32_t      traj_duration_ms = 500;
static bool          traj_active     = false;

// Smootherstep de Perlin: derivada zero em t=0 e t=1, sem overshooting
static float smootherstep(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

void trajectory_init() {
    traj_current  = {0.0f, 0.0f, 90.0f};
    traj_active   = false;
    servo_set(&traj_current);
}

void trajectory_set_target(const JointAngles_t *target, uint8_t speed) {
    traj_start        = traj_current;
    traj_end          = *target;
    traj_start_ms     = millis();
    // speed 0 = lento (2000ms), 255 = rápido (80ms)
    traj_duration_ms  = (uint32_t)map(speed, 0, 255, 2000, 80);
    traj_active       = true;
}

void trajectory_update() {
    if (!traj_active) return;

    uint32_t elapsed = millis() - traj_start_ms;

    if (elapsed >= traj_duration_ms) {
        traj_current = traj_end;
        traj_active  = false;
    } else {
        float t = (float)elapsed / (float)traj_duration_ms;
        float s = smootherstep(t);
        traj_current.coxa_deg  = lerp(traj_start.coxa_deg,  traj_end.coxa_deg,  s);
        traj_current.femur_deg = lerp(traj_start.femur_deg, traj_end.femur_deg, s);
        traj_current.tibia_deg = lerp(traj_start.tibia_deg, traj_end.tibia_deg, s);
    }

    servo_set(&traj_current);
}

bool trajectory_done() {
    return !traj_active;
}
