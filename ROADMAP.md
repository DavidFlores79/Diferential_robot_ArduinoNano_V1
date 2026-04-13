# Robot Build Roadmap

Step-by-step test sequence for the differential drive robot.
Each phase has its own sketch. Complete and validate each before moving to the next.

---

## Phase 1 — LM393 Encoder Interrupt Test ✅
**Sketch:** `phase1_encoder/phase1_encoder.ino`
**Hardware:** Arduino Nano + LM393 encoder module
**Validates:**
- Interrupt-based pulse counting on D2
- Speed calculation in pulses/second
- 104 capacitor filtering noise

**Pass criteria:** Counts increment cleanly when wheel spins by hand. No phantom counts at rest.

---

## Phase 2 — TB6612FNG Motor + Encoder Combined ✅
**Sketch:** `phase2_motor_encoder/phase2_motor_encoder.ino`
**Hardware:** Phase 1 + TB6612FNG + N20 motor + 2x 18650 battery
**Validates:**
- Motor forward/reverse ramp via PWM
- STBY enable/disable
- Encoder counting while motor drives wheel (not just manual spin)

**Wiring:**
```
Battery+  → TB6612 VM
Nano 5V   → TB6612 VCC
GND       → TB6612 GND (bridge both GND rails!)
Nano D4   → STBY
Nano D5   → PWMA
Nano D7   → AIN2
Nano D8   → AIN1
TB6612 AO1/AO2 → Motor
LM393 D0  → Nano D2
```

**Pass criteria:** Motor ramps up/down in both directions. Encoder count increases while motor runs. Serial prints speed correctly.

---

## Phase 3 — Closed-Loop Speed Control (PID)
**Sketch:** `phase3_pid_speed/phase3_pid_speed.ino`
**Hardware:** Same as Phase 2
**Validates:**
- PID controller regulates wheel speed to a target (pulses/s)
- Motor PWM adjusts automatically to maintain target speed
- Serial prints setpoint vs actual speed

**Pass criteria:** Wheel holds target speed under load. Minimal overshoot. Stable at low and high speeds.

---

## Phase 4 — Two Motors + Two Encoders
**Sketch:** `phase4_dual_motor/phase4_dual_motor.ino`
**Hardware:** Phase 3 x2 (second motor + encoder on D3, BIN1/BIN2/PWMB)
**Validates:**
- Both wheels independently PID-controlled
- Straight line drive (both wheels same speed)
- Basic differential turn (speed difference between wheels)

**Wiring additions:**
```
LM393 D0 (right) → Nano D3
Nano D6   → PWMB
Nano D9   → BIN1
Nano D10  → BIN2
TB6612 BO1/BO2 → Right motor
```

**Pass criteria:** Robot drives straight. Turns predictably by changing one wheel speed.

---

## Phase 5 — MPU-6050 IMU Read
**Sketch:** `phase5_imu/phase5_imu.ino`
**Hardware:** Phase 4 + MPU-6050 (I2C)
**Library:** `Wire.h` (built-in)
**Validates:**
- I2C communication with MPU-6050 (address 0x68)
- Raw accelerometer and gyroscope values
- Motion detection (lifted vs grounded)

**Wiring:**
```
MPU-6050 VCC → 3.3V
MPU-6050 GND → GND
MPU-6050 SDA → Nano A4
MPU-6050 SCL → Nano A5
MPU-6050 AD0 → GND (sets address to 0x68)
```

**Pass criteria:** Serial prints ax/ay/az and gx/gy/gz. Values change when board is tilted/moved.

---

## Phase 6 — HMC5883L Magnetometer + Yaw Fusion
**Sketch:** `phase6_yaw_fusion/phase6_yaw_fusion.ino`
**Hardware:** Phase 5 + HMC5883L (I2C, address 0x1E)
**Validates:**
- Magnetometer heading (0–360°)
- Gyroscope yaw integration
- Complementary filter fusing both into stable yaw estimate

**Wiring:**
```
HMC5883L VCC → 3.3V
HMC5883L GND → GND
HMC5883L SDA → Nano A4 (shared I2C bus)
HMC5883L SCL → Nano A5 (shared I2C bus)
```

**Pass criteria:** Yaw estimate follows robot rotation. Gyro drift corrected over time by magnetometer.

---

## Phase 7 — OLED Display + Expressions
**Sketch:** `phase7_oled/phase7_oled.ino`
**Hardware:** Phase 6 + SSD1306 OLED (I2C, address 0x3C)
**Library:** Adafruit SSD1306 + Adafruit GFX
**Validates:**
- OLED initialises and displays content
- Basic eye/expression bitmaps for each robot state
- State transitions reflected on screen

**Wiring:**
```
OLED VCC → 3.3V or 5V (check module spec)
OLED GND → GND
OLED SDA → Nano A4
OLED SCL → Nano A5
```

**Pass criteria:** Each robot state (IDLE, MOVING, LIFTED, ANGRY) shows a distinct expression.

---

## Phase 8 — Vibration Motor PWM Patterns
**Sketch:** `phase8_vibration/phase8_vibration.ino`
**Hardware:** Phase 7 + coin vibration motor + NPN transistor (e.g. 2N2222) or MOSFET
**Validates:**
- PWM intensity control of vibration motor
- Pattern sequences (pulse, ramp, burst)
- Non-blocking pattern playback

**Wiring:**
```
Nano D11 → transistor base (via 1kΩ resistor)
Transistor collector → vibration motor −
Vibration motor + → 5V
Transistor emitter → GND
Flyback diode across motor terminals
```

**Pass criteria:** Motor vibrates at different intensities. Patterns play without blocking other code.

---

## Phase 9 — Full Behavior State Machine
**Sketch:** `phase9_state_machine/phase9_state_machine.ino`
**Hardware:** All of the above
**Validates:**
- State transitions: IDLE → MOVING → LIFTED → INTERACTING → ANGRY
- All outputs (motors, OLED, vibration) respond to state
- All inputs (IMU, encoders, BT connection) drive state changes

**Pass criteria:** Robot reacts correctly to being picked up, driven, and interacted with. No blocking code anywhere.

---

## Phase 10 — ESP32 Port
**Sketch:** New project folder `Diferential_robot_ESP32_V1/`
**Changes from Nano:**
- `analogWrite()` → ESP32 LEDC (`ledcWrite()`)
- I2C pins: SDA=21, SCL=22 (default)
- Interrupt pins: any GPIO
- Add Bluetooth serial for remote control
- Add WiFi OTA for wireless uploads

**Pass criteria:** All Phase 9 functionality working on ESP32 with Bluetooth control.

---

## Quick Reference — Pin Map (Arduino Nano)

| Function | Pin |
|---|---|
| Encoder Left | D2 (INT0) |
| Encoder Right | D3 (INT1) |
| STBY | D4 |
| PWMA (left) | D5 |
| PWMB (right) | D6 |
| AIN2 | D7 |
| AIN1 | D8 |
| BIN1 | D9 |
| BIN2 | D10 |
| Vibration motor | D11 |
| I2C SDA | A4 |
| I2C SCL | A5 |
