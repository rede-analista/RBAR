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

## Roadmap

- [x] Cinemática inversa analítica
- [x] Interpolação de trajetória (smootherstep)
- [x] Controle de servos PWM
- [x] Protocolo I2C slave
- [ ] Cinemática direta (`fk_solve`) para validação
- [ ] Módulo de debug serial
- [ ] Testes em hardware
- [ ] Suporte a múltiplas pernas (endereços I2C configuráveis)
- [ ] Gerador de padrões de marcha (gait patterns)

---

## Licença

MIT
