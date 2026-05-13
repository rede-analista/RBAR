#include "i2c_slave.h"
#include "kinematics.h"
#include "trajectory.h"
#include "debug_serial.h"
#include <Wire.h>
#include <string.h>

volatile I2C_Command_t i2c_last_cmd  = CMD_NONE;
volatile Vec3_t        i2c_target_pos = {0.0f, 0.0f, -100.0f};
volatile uint8_t       i2c_speed     = 128;

static uint8_t rx_buf[16];
static uint8_t rx_len  = 0;
static bool    rx_ready = false;

// Chamado pela ISR do TWI — armazena bytes e sinaliza para o loop
static void on_receive(int bytes) {
    rx_len = 0;
    while (Wire.available() && rx_len < (uint8_t)sizeof(rx_buf)) {
        rx_buf[rx_len++] = (uint8_t)Wire.read();
    }
    rx_ready = true;
}

// Chamado quando o mestre faz um read request — envia ângulos atuais
static void on_request() {
    // Resposta: 3 floats (coxa, femur, tibia) = 12 bytes + 1 byte status
    uint8_t buf[13];
    float c = traj_current.coxa_deg;
    float f = traj_current.femur_deg;
    float t = traj_current.tibia_deg;
    memcpy(&buf[0], &c, 4);
    memcpy(&buf[4], &f, 4);
    memcpy(&buf[8], &t, 4);
    buf[12] = trajectory_done() ? 0x01 : 0x00; // 0x01 = movimento concluído
    Wire.write(buf, 13);
}

void i2c_init() {
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(on_receive);
    Wire.onRequest(on_request);
}

void i2c_process() {
    if (!rx_ready) return;
    rx_ready = false;

    if (rx_len == 0) return;

    I2C_Command_t cmd = (I2C_Command_t)rx_buf[0];

    switch (cmd) {
        case CMD_SET_POS:
            // Payload: 3 floats (12 bytes) = X, Y, Z em mm
            if (rx_len >= 13) {
                float x, y, z;
                memcpy(&x, &rx_buf[1], 4);
                memcpy(&y, &rx_buf[5], 4);
                memcpy(&z, &rx_buf[9], 4);

                Vec3_t pos = {x, y, z};
                JointAngles_t angles;
                bool ok = ik_solve(&pos, &angles);
                debug_ik_result(&pos, &angles, ok);
                if (ok) {
                    debug_fk_verify(&angles);
                    trajectory_set_target(&angles, i2c_speed);
                    i2c_target_pos = pos;
                }
            }
            break;

        case CMD_SET_SPEED:
            // Payload: 1 byte (0=lento, 255=rápido)
            if (rx_len >= 2) {
                i2c_speed = rx_buf[1];
            }
            break;

        case CMD_HOME: {
            JointAngles_t home = {0.0f, 0.0f, 90.0f};
            trajectory_set_target(&home, i2c_speed);
            break;
        }

        case CMD_STATUS:
            // Resposta enviada pelo on_request(); nada a fazer aqui
            break;

        default:
            break;
    }

    i2c_last_cmd = cmd;
}
