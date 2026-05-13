#include "debug_serial.h"

#ifdef DEBUG_SERIAL

#include "kinematics.h"
#include "trajectory.h"
#include "i2c_slave.h"
#include <Arduino.h>

#define DEBUG_BAUD     115200
#define DEBUG_INTERVAL 200   // ms entre impressões de estado

static uint32_t last_print_ms = 0;

void debug_init() {
    Serial.begin(DEBUG_BAUD);
    while (!Serial) {}
    Serial.println(F("=== RBAR Debug Serial ==="));
    Serial.print(F("Firmware: ")); Serial.println(F(FIRMWARE_VERSION));
    Serial.print(F("I2C addr: 0x")); Serial.println(I2C_ADDRESS, HEX);
    Serial.println(F("Geometria (mm):"));
    Serial.print(F("  Coxa=")); Serial.print(COXA_LEN);
    Serial.print(F(" Femur=")); Serial.print(FEMUR_LEN);
    Serial.print(F(" Tibia=")); Serial.println(TIBIA_LEN);
    Serial.println(F("-------------------------"));
}

// Imprime estado atual a cada DEBUG_INTERVAL ms
void debug_update() {
    uint32_t now = millis();
    if (now - last_print_ms < DEBUG_INTERVAL) return;
    last_print_ms = now;

    const JointAngles_t &a = traj_current;

    Serial.print(F("[t=")); Serial.print(now);
    Serial.print(F("ms] coxa="));   Serial.print(a.coxa_deg,  1);
    Serial.print(F(" femur="));     Serial.print(a.femur_deg, 1);
    Serial.print(F(" tibia="));     Serial.print(a.tibia_deg, 1);
    Serial.print(F(" done="));      Serial.println(trajectory_done() ? 1 : 0);
}

// Imprime resultado de uma chamada ik_solve
void debug_ik_result(const Vec3_t *target, const JointAngles_t *angles, bool ok) {
    Serial.print(F("[IK] target=("));
    Serial.print(target->x, 1); Serial.print(',');
    Serial.print(target->y, 1); Serial.print(',');
    Serial.print(target->z, 1); Serial.print(F(") -> "));

    if (!ok) {
        Serial.println(F("FORA DO ALCANCE"));
        return;
    }

    Serial.print(F("coxa="));  Serial.print(angles->coxa_deg,  1);
    Serial.print(F(" femur=")); Serial.print(angles->femur_deg, 1);
    Serial.print(F(" tibia=")); Serial.println(angles->tibia_deg, 1);
}

// Valida round-trip: FK(IK(pos)) deve reapresentar pos com erro < 1mm
void debug_fk_verify(const JointAngles_t *angles) {
    Vec3_t fk_result;
    fk_solve(angles, &fk_result);

    Serial.print(F("[FK] coxa="));  Serial.print(angles->coxa_deg,  1);
    Serial.print(F(" femur="));     Serial.print(angles->femur_deg, 1);
    Serial.print(F(" tibia="));     Serial.print(angles->tibia_deg, 1);
    Serial.print(F(" -> ("));
    Serial.print(fk_result.x, 2); Serial.print(',');
    Serial.print(fk_result.y, 2); Serial.print(',');
    Serial.print(fk_result.z, 2); Serial.println(')');
}

#endif
