/*
 * Phase 4 — Two Motors + Two Encoders (Independent PID)
 *
 * Each wheel has its own PID speed controller.
 * Tests straight driving and differential turning.
 *
 * Connections:
 * - Encoder Left  D0 → D2 (INT0)
 * - Encoder Right D0 → D3 (INT1)
 * - STBY → D4
 * - PWMA → D5 | AIN2 → D7 | AIN1 → D8  (left motor)
 * - PWMB → D6 | BIN1 → D9 | BIN2 → D10 (right motor)
 */

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
const float DRIVE_SPEED  = 40.0;  // p/s — straight drive speed
const float TURN_SPEED   = 30.0;  // p/s — speed of faster wheel during turn

// ============================================================================
// PID GAINS (tune these)
// ============================================================================
const float kP = 2.0;
const float kI = 0.5;
const float kD = 0.1;

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
struct PID {
  float target;
  float integral;
  float lastError;
  long  lastCount;
  float speed;
  int   pwmOut;
};

PID leftPID  = {0, 0, 0, 0, 0, 0};
PID rightPID = {0, 0, 0, 0, 0, 0};

// ============================================================================
// TIMING
// ============================================================================
unsigned long lastPIDTime    = 0;
unsigned long lastReportTime = 0;
unsigned long stateTimer     = 0;
const unsigned long PID_INTERVAL    = 100;
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
  pid.speed     = delta / dt;

  float error      = pid.target - pid.speed;
  pid.integral    += error * dt;
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

  setMotorLeft((int)computePID(leftPID, lc, dt));
  setMotorRight((int)computePID(rightPID, rc, dt));
}

// ============================================================================
// DRIVE STATE MACHINE
// ============================================================================
void updateDriveSequence(unsigned long now) {
  switch (driveState) {
    case FORWARD:
      leftPID.target  = DRIVE_SPEED;
      rightPID.target = DRIVE_SPEED;
      if (now - stateTimer >= 3000) {
        Serial.println(F(">> Turn left"));
        stateTimer = now;
        driveState = TURN_LEFT;
      }
      break;

    case TURN_LEFT:
      leftPID.target  = TURN_SPEED * 0.3;  // slow left
      rightPID.target = TURN_SPEED;
      if (now - stateTimer >= 1500) {
        Serial.println(F(">> Turn right"));
        stateTimer = now;
        driveState = TURN_RIGHT;
      }
      break;

    case TURN_RIGHT:
      leftPID.target  = TURN_SPEED;
      rightPID.target = TURN_SPEED * 0.3;  // slow right
      if (now - stateTimer >= 1500) {
        Serial.println(F(">> Stop"));
        stateTimer = now;
        driveState = STOP_PAUSE;
      }
      break;

    case STOP_PAUSE:
      leftPID.target  = 0;
      rightPID.target = 0;
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
  Serial.println(F("Sequence: Forward 3s → Turn left → Turn right → Stop → repeat"));
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
