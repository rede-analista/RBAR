#pragma once
#include <stdint.h>

// Lê os pinos de configuração de hardware e retorna o endereço I2C
// resultante. Deve ser chamado antes de Wire.begin().
//
// Pinos de endereço: PB4, PB5, PB6 com pull-up interno.
// Jumper ao GND = bit 0; aberto = bit 1.
// Endereço = I2C_ADDRESS_BASE | (A2<<2 | A1<<1 | A0)
// Suporta até 8 pernas (0x10–0x17).
uint8_t address_config_read();
