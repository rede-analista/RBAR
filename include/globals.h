#pragma once
#include <stdint.h>

#define FIRMWARE_VERSION "0.1.0"
#define I2C_ADDRESS      0x10

// Geometria da perna (mm)
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

typedef enum {
    CMD_NONE      = 0x00,
    CMD_SET_POS   = 0x01,
    CMD_SET_SPEED = 0x02,
    CMD_HOME      = 0x03,
    CMD_STATUS    = 0x04,
} I2C_Command_t;
