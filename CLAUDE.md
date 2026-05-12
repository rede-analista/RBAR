# CLAUDE.md — RBAR

## Visão geral

Firmware para ATmega324P que controla uma perna robótica com 3 graus de liberdade.
Recebe comandos de um ATmega mestre via I2C e executa cinemática inversa + movimentos suaves.

## Build

```bash
platformio run                   # compilar
platformio run --target upload   # gravar via USBasp
```

## Arquitetura dos módulos

| Módulo | Arquivo | Responsabilidade |
|--------|---------|-----------------|
| i2c_slave | src/i2c_slave.cpp | Recebe comandos do mestre; responde com estado atual |
| kinematics | src/kinematics.cpp | Converte XYZ → ângulos articulares (IK analítica) |
| trajectory | src/trajectory.cpp | Interpolação smootherstep entre posições |
| servo_control | src/servo_control.cpp | Escreve ângulos nos servos via biblioteca Servo |

## Fluxo de um comando SET_POS

```
I2C onReceive → rx_buf preenchido
loop() → i2c_process() → ik_solve() → trajectory_set_target()
loop() → trajectory_update() → servo_set()
loop() → servo_update() → Servo.write()
```

## Protocolo I2C

- Endereço slave: `0x10` (definido em `globals.h`)
- Byte 0 = comando, bytes seguintes = payload

| Comando | Código | Payload (bytes) |
|---------|--------|-----------------|
| SET_POS | 0x01 | X(4) Y(4) Z(4) — floats little-endian, mm |
| SET_SPEED | 0x02 | speed(1) — 0=lento(2s) 255=rápido(80ms) |
| HOME | 0x03 | — |
| STATUS | 0x04 | — (mestre faz read request; recebe 13 bytes) |

**Resposta ao read request (13 bytes):**
coxa_deg(4) + femur_deg(4) + tibia_deg(4) + status(1)
`status = 0x01` quando movimento concluído.

## Geometria (provisória — ajustar conforme hardware real)

| Parâmetro | Valor |
|-----------|-------|
| COXA_LEN | 50 mm |
| FEMUR_LEN | 80 mm |
| TIBIA_LEN | 100 mm |
| COXA faixa | -60° a +60° |
| FEMUR faixa | -90° a +90° |
| TIBIA faixa | 0° a +135° |

## Calibração de servos

Editar em `servo_control.cpp`:
- `OFFSET_COXA / OFFSET_FEMUR / OFFSET_TIBIA` — zero mecânico de cada servo
- `PIN_COXA / PIN_FEMUR / PIN_TIBIA` — pinos do ATmega324P
- `PULSE_MIN / PULSE_MAX` — largura de pulso (padrão 500–2500µs)

## Cinemática inversa

Solução analítica em 3 passos:
1. `theta_coxa = atan2(y, x)` — rotação horizontal
2. Projeção no plano fêmur-tíbia: `r = sqrt(x²+y²) - COXA_LEN`, `h = -z`
3. Lei dos cossenos para fêmur e tíbia

Retorna `false` se ponto fora do espaço de trabalho. Sempre clampear após IK.

## Interpolação de trajetória

Smootherstep de Perlin: `f(t) = 6t⁵ - 15t⁴ + 10t³`
Derivada zero em t=0 e t=1 → aceleração e desaceleração suaves, sem jerk.

## Convenções

- Ângulos IK: centrados em zero (coxa: -60..+60, femur: -90..+90)
- Ângulos servo: 0–180°, centro = 90° (coxa e femur), 0° (tibia)
- Coordenadas: X=frente, Y=lateral, Z=vertical (Z negativo = abaixo do corpo)
