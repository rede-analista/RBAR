#include "address_config.h"
#include "globals.h"
#include <Arduino.h>

// Pinos de configuração (escolhidos para não conflitar com I2C, UART e PWM)
#define ADDR_PIN_0  12   // PB4 — bit 0 do endereço
#define ADDR_PIN_1  13   // PB5 — bit 1
#define ADDR_PIN_2  14   // PB6 — bit 2

uint8_t address_config_read() {
    pinMode(ADDR_PIN_0, INPUT_PULLUP);
    pinMode(ADDR_PIN_1, INPUT_PULLUP);
    pinMode(ADDR_PIN_2, INPUT_PULLUP);

    // Aguarda estabilização dos pull-ups
    delay(1);

    // Pull-up ativo = pino aberto = bit 1; jumper ao GND = bit 0
    // Invertemos a leitura: LOW (jumper) = 1 no endereço
    uint8_t a0 = !digitalRead(ADDR_PIN_0);
    uint8_t a1 = !digitalRead(ADDR_PIN_1);
    uint8_t a2 = !digitalRead(ADDR_PIN_2);

    return (uint8_t)(I2C_ADDRESS_BASE | (a2 << 2) | (a1 << 1) | a0);
}
