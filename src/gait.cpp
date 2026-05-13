#include "gait.h"
#include "kinematics.h"
#include "servo_control.h"
#include "trajectory.h"
#include <Arduino.h>
#include <math.h>

GaitParams_t gait_params = {
    .neutral_x   = 120.0f,
    .neutral_y   =   0.0f,
    .ground_z    = -80.0f,
    .step_length =  60.0f,
    .step_height =  30.0f,
    .period_ms   = 1000,
    .phase       =   0.0f,
    .duty_cycle  =   0.6f,
};

static bool     active        = false;
static uint32_t cycle_start_ms = 0;

void gait_init() {
    active = false;
}

void gait_set_params(const GaitParams_t *params) {
    gait_params = *params;
}

void gait_start() {
    cycle_start_ms = millis();
    active = true;
}

void gait_stop() {
    active = false;
    JointAngles_t home = {0.0f, 0.0f, 90.0f};
    trajectory_set_target(&home, 128);
}

bool gait_running() {
    return active;
}

// Calcula a posição do pé em função da fase normalizada φ ∈ [0, 1)
static Vec3_t foot_position(float phi) {
    const GaitParams_t &g = gait_params;
    Vec3_t pos;

    if (phi < g.duty_cycle) {
        // Fase de apoio (stance): pé no chão, move de frente para trás
        float t = phi / g.duty_cycle;
        pos.x = g.neutral_x + g.step_length * 0.5f * (1.0f - 2.0f * t);
        pos.y = g.neutral_y;
        pos.z = g.ground_z;
    } else {
        // Fase de balanço (swing): pé levantado, move de trás para frente
        float t = (phi - g.duty_cycle) / (1.0f - g.duty_cycle);
        pos.x = g.neutral_x - g.step_length * 0.5f * (1.0f - 2.0f * t);
        pos.y = g.neutral_y;
        // Arco sinusoidal: sobe até step_height no meio do swing
        pos.z = g.ground_z - g.step_height * sinf(M_PI * t);
    }

    return pos;
}

void gait_update() {
    if (!active) return;

    uint32_t elapsed = millis() - cycle_start_ms;

    // Fase normalizada considerando offset de fase configurado
    float phi = fmodf((float)elapsed / (float)gait_params.period_ms
                      + gait_params.phase, 1.0f);

    Vec3_t target = foot_position(phi);

    JointAngles_t angles;
    if (ik_solve(&target, &angles)) {
        servo_set(&angles);
    }
    // Se fora do alcance, mantém posição atual (sem travar)
}
