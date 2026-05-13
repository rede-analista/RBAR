#include "kinematics.h"
#include <math.h>

bool ik_solve(const Vec3_t *target, JointAngles_t *out) {
    float x = target->x;
    float y = target->y;
    float z = target->z;

    // Coxa: rotação horizontal em torno do eixo Z
    float coxa_rad = atan2f(y, x);

    // Distância horizontal do pivot do fêmur até o pé
    float r = sqrtf(x*x + y*y) - COXA_LEN;
    float h = -z; // positivo = abaixo do corpo

    // Distância euclidiana do pivot do fêmur até o pé
    float D = sqrtf(r*r + h*h);

    // Verificação de alcance
    if (D > (FEMUR_LEN + TIBIA_LEN)) return false;
    if (D < fabsf(FEMUR_LEN - TIBIA_LEN)) return false;
    if (D < 0.001f) return false;

    // Ângulo da linha fêmur→pé em relação à horizontal
    float phi = atan2f(h, r);

    // Lei dos cossenos: ângulo entre fêmur e linha D
    float cos_alpha = (FEMUR_LEN*FEMUR_LEN + D*D - TIBIA_LEN*TIBIA_LEN)
                      / (2.0f * FEMUR_LEN * D);
    cos_alpha = fmaxf(-1.0f, fminf(1.0f, cos_alpha));
    float alpha = acosf(cos_alpha);

    // Lei dos cossenos: ângulo interno fêmur-tíbia
    float cos_beta = (FEMUR_LEN*FEMUR_LEN + TIBIA_LEN*TIBIA_LEN - D*D)
                     / (2.0f * FEMUR_LEN * TIBIA_LEN);
    cos_beta = fmaxf(-1.0f, fminf(1.0f, cos_beta));
    float beta = acosf(cos_beta);

    out->coxa_deg  = coxa_rad * (180.0f / M_PI);
    out->femur_deg = (phi - alpha) * (180.0f / M_PI);
    out->tibia_deg = (M_PI - beta) * (180.0f / M_PI);

    ik_clamp(out);
    return true;
}

void fk_solve(const JointAngles_t *angles, Vec3_t *out) {
    float t1 = angles->coxa_deg  * (M_PI / 180.0f);
    float t2 = angles->femur_deg * (M_PI / 180.0f);
    float t3 = angles->tibia_deg * (M_PI / 180.0f);

    // Direção da tíbia no plano 2D (r, h): θ2 + θ3
    float tibia_dir = t2 + t3;

    // Posição do pé no plano 2D relativo ao pivot do fêmur
    float r_foot = FEMUR_LEN * cosf(t2) + TIBIA_LEN * cosf(tibia_dir);
    float h_foot = FEMUR_LEN * sinf(t2) + TIBIA_LEN * sinf(tibia_dir);

    // De volta ao espaço 3D
    float r_total = COXA_LEN + r_foot;
    out->x = r_total * cosf(t1);
    out->y = r_total * sinf(t1);
    out->z = -h_foot; // h positivo = abaixo do corpo; z negativo = abaixo
}

void ik_clamp(JointAngles_t *a) {
    auto clamp = [](float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    };
    a->coxa_deg  = clamp(a->coxa_deg,  COXA_MIN,  COXA_MAX);
    a->femur_deg = clamp(a->femur_deg, FEMUR_MIN, FEMUR_MAX);
    a->tibia_deg = clamp(a->tibia_deg, TIBIA_MIN, TIBIA_MAX);
}
