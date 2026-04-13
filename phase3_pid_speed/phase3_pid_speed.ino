/*
 * Phase 3 — Closed-Loop Speed Control (PID)
 *
 * Controls motor speed to a target using encoder feedback.
 *
 * Connections:
 * - Encoder Left D0 → D2 (INT0)
 * - STBY → D4 | PWMA → D5 | AIN2 → D7 | AIN1 → D8
 * - Battery+ → TB6612 VM | Nano 5V → TB6612 VCC
 */

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
const uint8_t ENCODER_LEFT_PIN = 2;
const uint8_t STBY = 4;
const uint8_t PWMA = 5;
const uint8_t AIN1 = 8;
const uint8_t AIN2 = 7;

// ============================================================================
// PID CONFIGURATION
// ============================================================================
float targetSpeed = 40.0;   // pulses/second — change to test different speeds

float kP = 2.0;
float kI = 0.5;
float kD = 0.1;

float integral   = 0;
float lastError  = 0;
int   motorPWM   = 0;

// ============================================================================
// ENCODER
// ============================================================================
volatile long encoderCount = 0;

void encoderISR() {
  encoderCount++;
}

// ============================================================================
// TIMING
// ============================================================================
unsigned long lastPIDTime    = 0;
unsigned long lastReportTime = 0;
const unsigned long PID_INTERVAL    = 100;   // PID runs every 100ms
const unsigned long REPORT_INTERVAL = 500;

long lastCount = 0;
float currentSpeed = 0;

// ============================================================================
// MOTOR
// ============================================================================
void motorSet(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwm);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -pwm);
  }
  motorPWM = pwm;
}

void motorStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  motorPWM = 0;
}

// ============================================================================
// PID UPDATE
// ============================================================================
void updatePID(unsigned long now) {
  if (now - lastPIDTime < PID_INTERVAL) return;

  float dt = (now - lastPIDTime) / 1000.0;  // seconds
  lastPIDTime = now;

  noInterrupts();
  long count = encoderCount;
  interrupts();

  long delta = count - lastCount;
  lastCount  = count;
  currentSpeed = delta / dt;

  float error = targetSpeed - currentSpeed;
  integral   += error * dt;
  float derivative = (error - lastError) / dt;
  lastError  = error;

  float output = (kP * error) + (kI * integral) + (kD * derivative);
  motorSet((int)output);
}

// ============================================================================
// REPORT
// ============================================================================
void printReport(unsigned long now) {
  if (now - lastReportTime < REPORT_INTERVAL) return;
  lastReportTime = now;

  Serial.print(F("Target: "));
  Serial.print(targetSpeed, 1);
  Serial.print(F(" p/s | Actual: "));
  Serial.print(currentSpeed, 1);
  Serial.print(F(" p/s | PWM: "));
  Serial.println(motorPWM);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("\n====================================="));
  Serial.println(F("Phase 3 — PID Speed Control"));
  Serial.println(F("====================================="));
  Serial.print(F("Target speed: "));
  Serial.print(targetSpeed);
  Serial.println(F(" p/s"));
  Serial.println(F("Change 'targetSpeed' in code to test different speeds."));
  Serial.println();

  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), encoderISR, CHANGE);

  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(STBY, HIGH);

  lastPIDTime    = millis();
  lastReportTime = millis();
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  updatePID(now);
  printReport(now);
}
