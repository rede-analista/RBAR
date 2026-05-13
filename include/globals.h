#pragma once
#include <stdint.h>

#define FIRMWARE_VERSION "0.3.0"

// Descomentar para habilitar saída de debug via UART (115200 baud)
// #define DEBUG_SERIAL

// Endereço I2C base — os 3 bits menos significativos são lidos dos pinos
// de configuração de hardware (PB4, PB5, PB6) em address_config.cpp
#define I2C_ADDRESS_BASE  0x10  // 0x10–0x17

// Geometria da perna (mm) — ajustar conforme hardware real
#define COXA_LEN    50.0f
#define FEMUR_LEN   80.0f
#define TIBIA_LEN   100.0f

// Limites articulares (graus)
#define COXA_MIN   -60
#define COXA_MAX    60
#define FEMUR_MIN  -90
#define FEMUR_MAX   90
#define TIBIA_MIN    0
#define TIBIA_MAX  135

typedef struct {
    float x, y, z;
} Vec3_t;

typedef struct {
    float coxa_deg;
    float femur_deg;
    float tibia_deg;
} JointAngles_t;

typedef struct {
    float    neutral_x;   // posição neutra X (mm, direção frontal)
    float    neutral_y;   // posição neutra Y (mm, offset lateral)
    float    ground_z;    // altura do chão (mm, negativo = abaixo do corpo)
    float    step_length; // comprimento do passo (mm)
    float    step_height; // altura do levantamento do pé (mm)
    uint16_t period_ms;   // duração de um ciclo completo (ms)
    float    phase;       // offset de fase 0.0–1.0 (para sincronismo entre pernas)
    float    duty_cycle;  // fração do ciclo em apoio (0.5–0.8)
} GaitParams_t;

typedef enum {
    CMD_NONE       = 0x00,
    CMD_SET_POS    = 0x01,
    CMD_SET_SPEED  = 0x02,
    CMD_HOME       = 0x03,
    CMD_STATUS     = 0x04,
    CMD_GAIT_SET   = 0x05,  // configura parâmetros de marcha
    CMD_GAIT_START = 0x06,  // inicia ciclo de marcha
    CMD_GAIT_STOP  = 0x07,  // para marcha e vai para home
} I2C_Command_t;
