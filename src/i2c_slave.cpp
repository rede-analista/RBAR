#include "i2c_slave.h"
#include "kinematics.h"
#include "trajectory.h"
#include "gait.h"
#include "debug_serial.h"
#include "address_config.h"
#include <Wire.h>
#include <string.h>

I2C_Command_t i2c_last_cmd   = CMD_NONE;
Vec3_t        i2c_target_pos = {0.0f, 0.0f, -100.0f};
uint8_t       i2c_speed      = 128;

static uint8_t rx_buf[32];
static uint8_t rx_len   = 0;
static bool    rx_ready = false;

static void on_receive(int bytes) {
    rx_len = 0;
    while (Wire.available() && rx_len < (uint8_t)sizeof(rx_buf)) {
        rx_buf[rx_len++] = (uint8_t)Wire.read();
    }
    rx_ready = true;
}

static void on_request() {
    uint8_t buf[14];
    float c = traj_current.coxa_deg;
    float f = traj_current.femur_deg;
    float t = traj_current.tibia_deg;
    memcpy(&buf[0], &c, 4);
    memcpy(&buf[4], &f, 4);
    memcpy(&buf[8], &t, 4);
    buf[12] = trajectory_done() ? 0x01 : 0x00;
    buf[13] = gait_running()    ? 0x02 : 0x00;
    Wire.write(buf, 14);
}

void i2c_init() {
    uint8_t addr = address_config_read();
    Wire.begin(addr);
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
            if (rx_len >= 13) {
                gait_stop(); // interrompe marcha se ativa
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
            if (rx_len >= 2) i2c_speed = rx_buf[1];
            break;

        case CMD_HOME:
            gait_stop();
            break;

        case CMD_STATUS:
            break;

        case CMD_GAIT_SET:
            // Payload: GaitParams_t serializado (30 bytes)
            if (rx_len >= 1 + sizeof(GaitParams_t)) {
                GaitParams_t params;
                memcpy(&params, &rx_buf[1], sizeof(GaitParams_t));
                gait_set_params(&params);
            }
            break;

        case CMD_GAIT_START:
            gait_start();
            break;

        case CMD_GAIT_STOP:
            gait_stop();
            break;

        default:
            break;
    }

    i2c_last_cmd = cmd;
}
