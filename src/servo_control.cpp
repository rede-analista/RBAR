#include "servo_control.h"
#include <Servo.h>
#include <Arduino.h>

// Pinos — ajustar conforme pinagem real do ATmega324P
#define PIN_COXA   9
#define PIN_FEMUR  10
#define PIN_TIBIA  11

// Offset mecânico de cada servo (graus) — calibrar em campo
#define OFFSET_COXA   0
#define OFFSET_FEMUR  0
#define OFFSET_TIBIA  0

// Pulso mínimo/máximo (µs) — servos padrão: 1000-2000µs
#define PULSE_MIN 500
#define PULSE_MAX 2500

static Servo     sv_coxa;
static Servo     sv_femur;
static Servo     sv_tibia;
static JointAngles_t sv_current;

// Converte ângulo IK (centrado em zero) para posição servo (0-180°)
static int deg_to_servo(float deg, int offset, int center = 90) {
    int val = (int)(center + deg) + offset;
    return val < 0 ? 0 : (val > 180 ? 180 : val);
}

void servo_init() {
    sv_coxa.attach(PIN_COXA,   PULSE_MIN, PULSE_MAX);
    sv_femur.attach(PIN_FEMUR, PULSE_MIN, PULSE_MAX);
    sv_tibia.attach(PIN_TIBIA, PULSE_MIN, PULSE_MAX);
    sv_current = {0.0f, 0.0f, 90.0f};
    servo_update();
}

void servo_set(const JointAngles_t *angles) {
    sv_current = *angles;
}

void servo_update() {
    sv_coxa.write(deg_to_servo(sv_current.coxa_deg,  OFFSET_COXA));
    sv_femur.write(deg_to_servo(sv_current.femur_deg, OFFSET_FEMUR));
    sv_tibia.write(deg_to_servo(sv_current.tibia_deg, OFFSET_TIBIA, 0));
}
