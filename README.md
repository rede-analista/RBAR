# RBAR — Robotic Biomorphic Articulated Robotics

Firmware para ATmega324P que controla uma perna robótica com **3 graus de liberdade**, projetado para movimentos fluidos e realistas inspirados na locomoção biológica.

---

## Objetivo

Desenvolver um firmware embarcado capaz de:

- Receber comandos de posição de um microcontrolador mestre via **I2C**
- Calcular automaticamente os ângulos articulares através de **cinemática inversa analítica**
- Executar **movimentos suaves** com aceleração e desaceleração naturais
- Respeitar os **limites mecânicos** de cada articulação
- Operar de forma **modular e otimizada** para as restrições do ATmega324P

---

## Hardware

| Componente | Especificação |
|------------|---------------|
| Microcontrolador | ATmega324P @ 16 MHz |
| Comunicação | I2C (slave, endereço `0x10`) |
| Atuadores | 3 servos PWM (padrão 50 Hz) |
| Toolchain | PlatformIO + framework Arduino |
| Gravação | USBasp |

### Perna robótica — 3 DoF

```
Corpo
  │
  ├── [Coxa]   rotação horizontal  (-60° a +60°)
  │
  ├── [Fêmur]  elevação vertical   (-90° a +90°)
  │
  └── [Tíbia]  extensão            (0° a +135°)
```

### Geometria padrão (ajustável em `globals.h`)

| Segmento | Comprimento |
|----------|-------------|
| Coxa | 50 mm |
| Fêmur | 80 mm |
| Tíbia | 100 mm |

---

## Arquitetura do firmware

```
main.cpp
├── i2c_slave       Recebe comandos do mestre; responde com estado atual
├── kinematics      Cinemática inversa analítica (XYZ → ângulos)
├── trajectory      Interpolação smootherstep de Perlin
└── servo_control   Geração de PWM para os 3 servos
```

### Fluxo de execução

```
[Mestre I2C] ──SET_POS(x,y,z)──► i2c_slave
                                      │
                                  ik_solve()
                                      │
                                  trajectory_set_target()
                                      │
                              loop() → trajectory_update()
                                      │
                              smootherstep interpolation
                                      │
                              servo_update() → PWM
```

---

## Protocolo I2C

**Endereço slave:** `0x10`

### Comandos (mestre → escravo)

| Comando | Código | Payload |
|---------|--------|---------|
| `SET_POS` | `0x01` | X(4 bytes) + Y(4 bytes) + Z(4 bytes) — floats, little-endian, mm |
| `SET_SPEED` | `0x02` | speed (1 byte) — `0`=lento (2 s) / `255`=rápido (80 ms) |
| `HOME` | `0x03` | — retorna para posição neutra |
| `STATUS` | `0x04` | — sem payload; mestre faz read request na sequência |

### Resposta ao STATUS (13 bytes)

```
[ coxa_deg (4B) | femur_deg (4B) | tibia_deg (4B) | status (1B) ]
status: 0x00 = em movimento | 0x01 = movimento concluído
```

---

## Cinemática inversa

Solução analítica em 3 passos — sem iteração, custo fixo de CPU:

1. **Coxa:** `θ₁ = atan2(y, x)`
2. **Projeção 2D:** `r = √(x²+y²) − L_coxa`, `h = −z`
3. **Fêmur e Tíbia** via lei dos cossenos:

```
D = √(r² + h²)
θ₂ = atan2(h, r) − acos((L_f² + D² − L_t²) / (2·L_f·D))
θ₃ = π − acos((L_f² + L_t² − D²) / (2·L_f·L_t))
```

Retorna `false` se o ponto estiver fora do espaço de trabalho.

---

## Interpolação de trajetória

**Smootherstep de Perlin:** `f(t) = 6t⁵ − 15t⁴ + 10t³`

Derivada zero em `t=0` e `t=1` — partida e parada sem jerk, simulando a resposta muscular biológica. Velocidade configurável em runtime via `SET_SPEED`.

---

## Debug serial

Para habilitar saída diagnóstica via UART, descomentar em `include/globals.h`:

```cpp
#define DEBUG_SERIAL
```

Conectar um adaptador USB-Serial (FTDI) ao ATmega324P e monitorar:

```bash
platformio device monitor --baud 115200
```

**Saída de exemplo:**
```
=== RBAR Debug Serial ===
Firmware: 0.2.0
I2C addr: 0x10
Geometria (mm): Coxa=50 Femur=80 Tibia=100
[IK] target=(120.0,0.0,-80.0) -> coxa=0.0 femur=-23.4 tibia=67.8
[FK] coxa=0.0 femur=-23.4 tibia=67.8 -> (120.00,0.00,-80.01)
[t=1200ms] coxa=0.0 femur=-23.4 tibia=67.8 done=1
```

O `debug_fk_verify` valida o round-trip `FK(IK(pos)) ≈ pos` — erro < 0.1 mm confirma que a cinemática está correta.

---

## Build e gravação

```bash
# Compilar
platformio run

# Gravar via USBasp
platformio run --target upload

# Monitor serial (se DEBUG_SERIAL habilitado)
platformio device monitor --baud 115200
```

---

## Calibração

Editar `src/servo_control.cpp`:

```cpp
#define PIN_COXA    9    // ajustar conforme pinagem real
#define PIN_FEMUR   10
#define PIN_TIBIA   11

#define OFFSET_COXA   0  // offset mecânico em graus
#define OFFSET_FEMUR  0
#define OFFSET_TIBIA  0

#define PULSE_MIN  500   // µs — ajustar para os servos utilizados
#define PULSE_MAX  2500
```

Editar `include/globals.h` para ajustar a geometria real da perna:

```cpp
#define COXA_LEN   50.0f  // mm
#define FEMUR_LEN  80.0f
#define TIBIA_LEN  100.0f
```

---

## Endereço I2C configurável

O endereço I2C é definido por hardware através de 3 pinos (PB4, PB5, PB6) com pull-up interno. Um jumper conectado ao GND define o bit como `1`; pino aberto = bit `0`.

| PB6 | PB5 | PB4 | Endereço | Perna sugerida |
|-----|-----|-----|----------|----------------|
| 0 | 0 | 0 | 0x10 | Frente-Esquerda |
| 0 | 0 | 1 | 0x11 | Frente-Direita |
| 0 | 1 | 0 | 0x12 | Meio-Esquerda |
| 0 | 1 | 1 | 0x13 | Meio-Direita |
| 1 | 0 | 0 | 0x14 | Trás-Esquerda |
| 1 | 0 | 1 | 0x15 | Trás-Direita |

## Padrões de marcha (Gait)

O módulo de marcha gera ciclos autônomos de movimento sem intervenção do mestre após o `GAIT_START`.

**Fases do ciclo:**

```
Fase de apoio (stance, 0→duty_cycle):
  pé no chão, move linearmente da frente para trás

Fase de balanço (swing, duty_cycle→1.0):
  pé levantado, arco sinusoidal de trás para frente
  z = ground_z - step_height × sin(π × t_swing)
```

**Parâmetros configuráveis via `CMD_GAIT_SET`:**

| Campo | Padrão | Descrição |
|-------|--------|-----------|
| `neutral_x` | 120 mm | Posição X neutra da perna |
| `neutral_y` | 0 mm | Offset lateral |
| `ground_z` | -80 mm | Altura do chão |
| `step_length` | 60 mm | Comprimento do passo |
| `step_height` | 30 mm | Altura do levantamento do pé |
| `period_ms` | 1000 ms | Duração de um ciclo completo |
| `phase` | 0.0 | Offset de fase (0.0–1.0) para sincronizar pernas |
| `duty_cycle` | 0.6 | Fração do ciclo em apoio |

O campo `phase` permite que o mestre envie parâmetros diferentes para cada perna e coordene uma marcha hexápode (tripod, wave, ripple).

**Resposta ao STATUS (14 bytes):**
```
coxa_deg(4) + femur_deg(4) + tibia_deg(4) + move_done(1) + gait_active(1)
```

## Roadmap

- [x] Cinemática inversa analítica
- [x] Interpolação de trajetória (smootherstep)
- [x] Controle de servos PWM
- [x] Protocolo I2C slave
- [x] Cinemática direta (`fk_solve`) para validação
- [x] Módulo de debug serial (ativado por `#define DEBUG_SERIAL`)
- [ ] Testes em hardware
- [x] Suporte a múltiplas pernas (endereço I2C por jumper, 0x10–0x17)
- [x] Gerador de padrões de marcha (stance + swing sinusoidal, phase offset)

---

## Licença

MIT
