/*
 * Phase 4 — Two Motors + Two Encoders (Independent PID)
 *
 * Each wheel has its own PID speed controller.
 * Tests straight driving and differential turning.
 *
 * Connections:
 * - Encoder Left  → D2 (INT0)
 * - Encoder Right → D3 (INT1)
 * - STBY → D4
 * - PWMA → D5 | AIN2 → D7 | AIN1 → D8  (left motor)
 * - PWMB → D6 | BIN1 → D9 | BIN2 → D10 (right motor)
 */

#include "phase4_dual_motor.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
const uint8_t ENC_LEFT  = 2;
const uint8_t ENC_RIGHT = 3;

const uint8_t STBY = 4;
const uint8_t PWMA = 5;   // Left
const uint8_t PWMB = 6;   // Right
const uint8_t AIN1 = 8;
const uint8_t AIN2 = 7;
const uint8_t BIN1 = 9;
const uint8_t BIN2 = 10;

// ============================================================================
// TEST SEQUENCE — edit these to change behaviour
// ============================================================================
const float DRIVE_SPEED  = 6.0;  // p/s — straight drive speed
const float TURN_SPEED   = 5.0;  // p/s — speed of faster wheel during turn

// ============================================================================
// PID GAINS (tune these)
// ============================================================================
float kP = 1.0;
float kI = 1.5;
float kD = 0.1;

// kI × INTEGRAL_MAX = max integral PWM contribution — must exceed steady-state PWM (~110)
// 1.5 × 80 = 120 → just enough headroom (10 PWM margin over ~110 needed)
const float INTEGRAL_MAX = 80.0;

// ============================================================================
// ENCODER
// ============================================================================
volatile long encLeftCount  = 0;
volatile long encRightCount = 0;

void encLeftISR()  { encLeftCount++; }
void encRightISR() { encRightCount++; }

// ============================================================================
// PID STATE
// ============================================================================
PID leftPID  = {0, 0, 0, 0, 0, 0};
PID rightPID = {0, 0, 0, 0, 0, 0};

// ============================================================================
// TIMING
// ============================================================================
unsigned long lastPIDTime    = 0;
unsigned long lastReportTime = 0;
unsigned long stateTimer     = 0;
const unsigned long PID_INTERVAL    = 200;  // 200ms → ~8 pulses/window at 40 p/s, reduces quantization noise
const unsigned long REPORT_INTERVAL = 500;

// ============================================================================
// TEST STATE MACHINE
// ============================================================================
enum DriveState { FORWARD, TURN_LEFT, TURN_RIGHT, STOP_PAUSE, DONE };
DriveState driveState = FORWARD;

// ============================================================================
// MOTOR
// ============================================================================
void setMotorLeft(int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(AIN1, pwm >= 0 ? HIGH : LOW);
  digitalWrite(AIN2, pwm >= 0 ? LOW  : HIGH);
  analogWrite(PWMA, abs(pwm));
  leftPID.pwmOut = pwm;
}

void setMotorRight(int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(BIN1, pwm >= 0 ? HIGH : LOW);
  digitalWrite(BIN2, pwm >= 0 ? LOW  : HIGH);
  analogWrite(PWMB, abs(pwm));
  rightPID.pwmOut = pwm;
}

// ============================================================================
// PID UPDATE
// ============================================================================
float computePID(PID &pid, long currentCount, float dt) {
  long delta    = currentCount - pid.lastCount;
  pid.lastCount = currentCount;
  float rawSpeed = delta / dt;
  if (rawSpeed > 30.0f) rawSpeed = pid.speed;  // discard noise spikes from PWM switching
  pid.speed = 0.6f * pid.speed + 0.4f * rawSpeed;  // EMA: reduce pulse-count noise

  float error      = pid.target - pid.speed;
  pid.integral    += error * dt;
  pid.integral     = constrain(pid.integral, -INTEGRAL_MAX, INTEGRAL_MAX);  // anti-windup
  float derivative = (error - pid.lastError) / dt;
  pid.lastError    = error;

  return (kP * error) + (kI * pid.integral) + (kD * derivative);
}

void updatePID(unsigned long now) {
  if (now - lastPIDTime < PID_INTERVAL) return;
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;

  noInterrupts();
  long lc = encLeftCount;
  long rc = encRightCount;
  interrupts();

  setMotorLeft((int)constrain(computePID(leftPID, lc, dt), 0, 255));
  setMotorRight((int)constrain(computePID(rightPID, rc, dt), 0, 255));
}

// ============================================================================
// DRIVE STATE MACHINE
// ============================================================================
void setTargets(float l, float r) {
  if (l != leftPID.target) {
    leftPID.integral = 0; leftPID.lastError = 0; leftPID.speed = 0;
    setMotorLeft(0);
  }
  if (r != rightPID.target) {
    rightPID.integral = 0; rightPID.lastError = 0; rightPID.speed = 0;
    setMotorRight(0);
  }
  leftPID.target  = l;
  rightPID.target = r;
}

void updateDriveSequence(unsigned long now) {
  switch (driveState) {
    case FORWARD:
      setTargets(DRIVE_SPEED, DRIVE_SPEED);
      if (now - stateTimer >= 7000) {
        Serial.println(F(">> Turn left"));
        stateTimer = now;
        driveState = TURN_LEFT;
      }
      break;

    case TURN_LEFT:
      setTargets(TURN_SPEED * 0.3, TURN_SPEED);
      if (now - stateTimer >= 2500) {
        Serial.println(F(">> Turn right"));
        stateTimer = now;
        driveState = TURN_RIGHT;
      }
      break;

    case TURN_RIGHT:
      setTargets(TURN_SPEED, TURN_SPEED * 0.3);
      if (now - stateTimer >= 2500) {
        Serial.println(F(">> Stop"));
        stateTimer = now;
        driveState = STOP_PAUSE;
      }
      break;

    case STOP_PAUSE:
      setTargets(0, 0);
      if (now - stateTimer >= 2000) {
        Serial.println(F(">> Forward"));
        stateTimer = now;
        driveState = FORWARD;
      }
      break;

    case DONE:
      break;
  }
}

// ============================================================================
// REPORT
// ============================================================================
void printReport(unsigned long now) {
  if (now - lastReportTime < REPORT_INTERVAL) return;
  lastReportTime = now;

  Serial.print(F("L: "));
  Serial.print(leftPID.speed, 1);
  Serial.print(F("/"));
  Serial.print(leftPID.target, 1);
  Serial.print(F(" p/s (PWM "));
  Serial.print(leftPID.pwmOut);
  Serial.print(F(") | R: "));
  Serial.print(rightPID.speed, 1);
  Serial.print(F("/"));
  Serial.print(rightPID.target, 1);
  Serial.print(F(" p/s (PWM "));
  Serial.print(rightPID.pwmOut);
  Serial.println(F(")"));
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("\n====================================="));
  Serial.println(F("Phase 4 — Dual Motor PID Control"));
  Serial.println(F("=====================================\n"));
  Serial.println(F("Sequence: Forward 7s → Turn left 2.5s → Turn right 2.5s → Stop → repeat"));
  Serial.println();

  pinMode(ENC_LEFT,  INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT),  encLeftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), encRightISR, CHANGE);

  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  lastPIDTime    = millis();
  lastReportTime = millis();
  stateTimer     = millis();

  Serial.println(F(">> Forward"));
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  updateDriveSequence(now);
  updatePID(now);
  printReport(now);
}
