# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project: Differential Drive Robot

An autonomous + reactive robot using ESP32 as the final target platform. **The current codebase is a test bench on Arduino Nano** — hardware modules are validated individually before integration into the full ESP32 firmware.

### Current Phase
Testing LM393 optical encoder interrupt counting (`Diferential_robot_ArduinoNano_V1.ino`). Each new subsystem starts as a standalone `.ino` test sketch before being promoted to a module in the final architecture.

---

## Hardware Target

| Component | Detail |
|---|---|
| MCU (test) | Arduino Nano (ATmega328P) |
| MCU (final) | ESP32 |
| Motor driver | TB6612FNG |
| Motors | 2× N20 DC gear motors |
| Encoders | LM393 optical comparator, ~20 pulses/revolution (shaft, before gearbox) |
| IMU | MPU-6050 (I2C) |
| Magnetometer | HMC5883L (I2C) |
| Display | OLED (expressive state feedback) |
| Haptics | Coin vibration motor (PWM) |
| Battery | 2× 18650 → LM2596 step-down → 5V |

---

## Encoder Wiring (Current Test)

```
Left Encoder  → D2 (INT0)
Right Encoder → D3 (INT1)
```

Both pins use `INPUT_PULLUP`. Interrupts trigger on `CHANGE` (counts both rising and falling edges, so 20-pulse discs yield 40 counts/revolution).

---

## Firmware Conventions

### Non-blocking mandatory
Never use `delay()`. All timing is `millis()`-based with `unsigned long` deltas. This applies to both Nano test sketches and the final ESP32 firmware.

### ISR rules
- ISR-shared variables must be `volatile`.
- Always wrap reads of volatile counters with `noInterrupts()` / `interrupts()`.
- Keep ISRs minimal — only increment counters, set flags.

### ESP32 motor control
Final firmware uses **ESP32 LEDC** for PWM (not `analogWrite`). Direction is set via AIN1/AIN2 and BIN1/BIN2. STBY pin enables/disables the TB6612FNG.

### Serial output
`115200` baud. Use `F()` macro for string literals on Nano to save SRAM. On ESP32, `F()` is a no-op but harmless.

---

## Planned Software Architecture (ESP32 target)

```
Layer 3 – Behavior
  State machine: IDLE | MOVING | LIFTED | INTERACTING | ANGRY
  Inputs: IMU motion, encoder activity, BT connection, touch
  Outputs: motors, OLED expressions, vibration patterns

Layer 2 – Control
  Wheel speed PID  (encoder feedback)
  Heading PID      (IMU/magnetometer yaw correction)
  Sensor fusion    (gyro short-term + magnetometer long-term)
  Odometry         (left+right pulse counts → Δdistance, Δtheta)

Layer 1 – Hardware Abstraction
  Motor driver     (TB6612FNG via LEDC PWM)
  Encoder reader   (interrupt-based pulse counting)
  IMU interface    (MPU-6050 I2C)
  Magnetometer     (HMC5883L I2C)
  OLED display
  Vibration motor  (PWM intensity + patterns)
```

Each layer should live in its own `.h`/`.cpp` pair when ported to ESP32.

---

## Development Workflow

There is no build CLI — compilation and upload happen through the **Arduino IDE** or **arduino-cli**.

### arduino-cli (if installed)
```bash
# Compile for Nano
arduino-cli compile --fqbn arduino:avr:nano Diferential_robot_ArduinoNano_V1

# Upload (adjust port)
arduino-cli upload -p COM3 --fqbn arduino:avr:nano Diferential_robot_ArduinoNano_V1

# Compile for ESP32
arduino-cli compile --fqbn esp32:esp32:esp32 <sketch_folder>
```

### Serial monitoring
```bash
arduino-cli monitor -p COM3 -c baudrate=115200
```

### No automated test suite
Validation is done by reading Serial output while manually spinning wheels or moving sensors. Each test sketch prints a clear human-readable report.

---

## Key Design Decisions

- **CHANGE-mode interrupts** on encoders: counts both edges → doubles effective resolution (40 counts/rev on 20-slot disc).
- **Odometry is dead-reckoning only** — encoder drift is expected; no absolute position correction planned in the near term.
- **Yaw fusion**: gyroscope for short-term stability + HMC5883L for long-term drift correction (complementary or Kalman filter TBD).
- **Vibration motor** is PWM-controlled for variable intensity, not just on/off.
