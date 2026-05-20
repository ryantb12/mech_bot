# Wiring & Components Reference

> This is the **single source of truth** for all physical connections.
> Update this file whenever you add, remove, or rewire a component.
> Then update `config/pins.py` to match.

---

## Inter-Board Communication
| From | To | Interface | Pins (From side) | Pins (To side) | Notes |
|---|---|---|---|---|---|
| T-Display S3 #1 (master) | T-Display S3 #2 (slave) | GPIO pulse | GPIO 17 (SLAVE_TX) | GPIO 16 (COMMS_PIN, INPUT_PULLDOWN) | Rising-edge pulse advances state |
| T-Display S3 #1 (master) | MOTION 2350 Pro | GPIO pulse | GPIO 1 (STATE_OUT_PIN) | GP26 (INPUT_PULLDOWN) | Rising-edge pulse advances state — confirmed wiring |
| T-Display S3 #1 (master) | MOTION 2350 Pro | UART TX | GPIO 14 (MOTION_UART_TX) | GP8 (Serial1 RX) | Panel servo commands: EXT:xxxx / FLIP:xxxx / NEUTRAL |

---

## Sensors

### HC-SR04 — Ultrasonic Distance
- **Connected to board:** T-Display S3 #1 (master)
- **Interface:** GPIO (trigger/echo)
- **Pins:**
  | Signal | Board GPIO |
  |---|---|
  | TRIG | 2 |
  | ECHO | 3 |

### MPU-6050 — IMU (Accelerometer + Gyroscope)
- **Connected to board:** T-Display S3 #1 (master)
- **Interface:** I2C
- **Pins:**
  | Signal | Board GPIO |
  |---|---|
  | SDA | 17 |
  | SCL | 18 |
- **I2C address:** `0x68`

---

## Actuators — T-Display S3 #2 (Slave Board)

> PWM uses ESP32-S3 LEDC peripheral — use `ledcSetup` / `ledcAttachPin` / `ledcWrite`.
> Do NOT use `analogWrite` on this board.

### Left Wheel Motor — MD1 Secondary (U4), A1/2 side
- **In1:** GPIO 43 (TTGO pin 3)
- **In2:** GPIO 44 (TTGO pin 4)
- **EN (PWM):** GPIO 10 (TTGO pin 20) — LEDC channel 0
- **Forward direction:** In1=LOW, In2=HIGH (dir = -1 in code)
- **Reverse direction:** In1=HIGH, In2=LOW (dir = 1 in code)

### Right Wheel Motor — MD2 Primary (U3), A1/2 side
- **In1:** GPIO 7 (TTGO pin 8)
- **In2:** GPIO 8 (TTGO pin 7)
- **EN (PWM):** GPIO 3 (TTGO pin 21) — LEDC channel 2
- **Forward direction:** In1=LOW, In2=HIGH (dir = -1 in code)
- **Reverse direction:** In1=LOW, In2=HIGH (dir = -1 in code — same as forward; right motor wired inverse to left)

### Linear Actuator — MD1 Secondary (U4), B1/2 side
- **In1:** GPIO 1 (TTGO pin 23)
- **In2:** GPIO 2 (TTGO pin 22)
- **EN:** GPIO 11 (TTGO pin 19) — must be driven HIGH to enable, LOW to disable
- **Extend (up):** In1=HIGH, In2=LOW, En=HIGH
- **Retract (down):** In1=LOW, In2=HIGH, En=HIGH
- **Stop:** In1=LOW, In2=LOW, En=LOW

---

## Master Board Pins — T-Display S3 #1

| Function | GPIO | Notes |
|---|---|---|
| TFT backlight | 15 | Set HIGH in setup |
| Ultrasonic A TRIG | 2 | Front-facing HC-SR04 |
| Ultrasonic A ECHO | 3 | Front-facing HC-SR04 |
| Ultrasonic B TRIG | 43 | Second HC-SR04 |
| Ultrasonic B ECHO | 44 | Second HC-SR04 |
| MPU-6050 SDA | 16 | |
| MPU-6050 SCL | 21 | |
| Button | 10 | INPUT_PULLUP, active LOW |
| STATE_OUT_PIN (to MOTION board) | 1 | GPIO pulse out |
| SLAVE_TX (to slave T-Display RX) | 17 | GPIO pulse out |
| SLAVE_RX (from slave T-Display TX) | 18 | Future use |
| Encoder motor IN1 | 11 | Arm extend |
| Encoder motor IN2 | 12 | Arm extend |
| Encoder motor EN | 13 | LEDC channel 4 |

---

## Power Distribution
| Rail | Source | Connected to | Notes |
|---|---|---|---|
| VIN | Battery / PSU | MOTION 2350 Pro VIN | Motor supply |
| 3.3 V | Each board onboard reg | GPIO logic, sensors | Max 300 mA per board |
| USB 5 V | USB-C | T-Display S3 #1, #2 | Development / programming only |

---

## Change Log
| Date | Change |
|---|---|
| 2026-05-19 | Filled in confirmed working slave board pins from testing_movement_v1 |
| 2026-05-19 | Added master board pin table, sensors, inter-board comms |
