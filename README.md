# Differential Drive Robot — Arduino Nano V1

Incremental build of a differential drive robot with autonomous and reactive behavior.
Each subsystem is validated on **Arduino Nano** as a standalone test sketch before being integrated into the final **ESP32** firmware.

---

## Hardware

| Component | Detail |
|---|---|
| MCU (test phase) | Arduino Nano (ATmega328P) |
| MCU (final) | ESP32 |
| Motor driver | TB6612FNG |
| Motors | 2× N20 DC gear motors |
| Encoders | LM393 optical comparator — ~20 pulses/rev (shaft, before gearbox) |
| IMU | MPU-6050 — accelerometer + gyroscope (I2C) |
| Magnetometer | HMC5883L — compass heading (I2C) |
| Display | SSD1306 OLED 128×64 — expressive eyes per robot state |
| Haptics | Coin vibration motor — PWM intensity + patterns |
| Battery | 2× 18650 in series (7.4V nominal) → LM2596 step-down → 5V logic rail |

---

## Power System

```
2× 18650 (7.4–8.4V)
    │
    ├──→ TB6612 VM  (raw battery voltage → motors)
    │
    └──→ LM2596 step-down
              │
              └──→ 5V rail
                      ├──→ Arduino Nano (5V pin)
                      ├──→ TB6612 VCC (logic)
                      └──→ LM393, sensors
```

> During bench testing: Nano powered via USB, 5V shield rail powers TB6612 VCC, battery powers TB6612 VM directly.

---

## Pin Map (Arduino Nano)

| Function | Pin |
|---|---|
| Encoder Left | D2 (INT0) |
| Encoder Right | D3 (INT1) |
| STBY (TB6612 enable) | D4 |
| PWMA — Left motor speed | D5 |
| PWMB — Right motor speed | D6 |
| AIN2 — Left direction | D7 |
| AIN1 — Left direction | D8 |
| BIN1 — Right direction | D9 |
| BIN2 — Right direction | D10 |
| Vibration motor | D11 |
| I2C SDA (MPU, HMC, OLED) | A4 |
| I2C SCL (MPU, HMC, OLED) | A5 |

---

## Build Roadmap

Each phase has its own sketch. Complete and validate each before moving to the next.

| Phase | Sketch | Status | Validates |
|---|---|---|---|
| 1 | `phase1_encoder/` | ✅ Done | LM393 interrupt counting, noise filtering |
| 2 | `phase2_motor_encoder/` | ✅ Done | TB6612 PWM control + encoder feedback |
| 3 | `phase3_pid_speed/` | ⬜ Next | Closed-loop speed PID with encoder |
| 4 | `phase4_dual_motor/` | ⬜ | Dual independent PID, straight drive, turns |
| 5 | `phase5_imu/` | ⬜ | MPU-6050 accel + gyro, lifted detection |
| 6 | `phase6_yaw_fusion/` | ⬜ | Gyro + magnetometer complementary filter |
| 7 | `phase7_oled/` | ⬜ | OLED expressions per robot state |
| 8 | `phase8_vibration/` | ⬜ | Non-blocking vibration pattern player |
| 9 | `phase9_state_machine/` | ⬜ | Full behavior integration |
| 10 | New ESP32 project | ⬜ | Port everything, add Bluetooth + OTA |

---

## Robot States

| State | Trigger | Motors | OLED | Vibration |
|---|---|---|---|---|
| IDLE | Default | Stopped | Normal eyes | Slow pulse |
| MOVING | Encoder activity | Running | Forward-looking eyes | Soft rhythm |
| LIFTED | IMU low-G detection | Stopped | Wide surprised eyes | Alert burst |
| INTERACTING | Touch / BT input | Stopped | Happy squint | Gentle ramp |
| ANGRY | Lifted too long | Stopped | Angry brow | Fast burst |

---

## Software Architecture

```
Layer 3 — Behavior
  State machine: IDLE | MOVING | LIFTED | INTERACTING | ANGRY
  Inputs:  IMU motion, encoder activity, Bluetooth, touch
  Outputs: motors, OLED expressions, vibration patterns

Layer 2 — Control
  Wheel speed PID     (encoder feedback, 100ms loop)
  Heading PID         (IMU/magnetometer yaw correction)
  Sensor fusion       (complementary filter: gyro + magnetometer)
  Odometry            (left + right pulses → Δdistance, Δtheta)

Layer 1 — Hardware Abstraction
  Motor driver        (TB6612FNG via PWM)
  Encoder reader      (interrupt-based, CHANGE mode = 40 counts/rev)
  IMU interface       (MPU-6050 raw I2C)
  Magnetometer        (HMC5883L raw I2C, shared bus)
  OLED display        (Adafruit SSD1306)
  Vibration motor     (PWM + non-blocking pattern player)
```

---

## Encoder Notes

- Interrupts trigger on `CHANGE` (both rising and falling edges)
- 20-slot disc × 2 edges = **40 counts/revolution** on motor shaft
- ISR variables are `volatile` — reads wrapped with `noInterrupts()` / `interrupts()`
- 104 capacitor (100nF) between D0 and GND filters signal noise

---

## TB6612FNG Direction Logic

| AIN1 | AIN2 | Result |
|---|---|---|
| HIGH | LOW | Forward |
| LOW | HIGH | Reverse |
| LOW | LOW | Coast |
| HIGH | HIGH | Brake |

---

## Libraries Required

| Library | Used in |
|---|---|
| `Wire.h` | Built-in — IMU, magnetometer, OLED |
| `Adafruit SSD1306` | Phase 7, 9 — OLED display |
| `Adafruit GFX` | Phase 7, 9 — OLED graphics |

Install Adafruit libraries via **Arduino IDE → Library Manager**.

---

## Future (Phase 10 — ESP32)

- Replace `analogWrite()` with ESP32 LEDC (`ledcWrite()`)
- I2C: SDA=21, SCL=22
- Bluetooth Serial for remote control
- WiFi OTA for wireless uploads
- Kalman filter replacing complementary filter for yaw
- Obstacle detection (ToF or ultrasonic)
- Waypoint-based autonomous navigation
