/*
 * LM393 Encoder + TB6612FNG Motor Test - Arduino Nano
 *
 * Encoder connections:
 * - Left Encoder D0 → D2 (INT0)
 *
 * Motor driver connections:
 * - STBY → D4
 * - PWMA → D5
 * - AIN2 → D7
 * - AIN1 → D8
 * - AO1/AO2 → Motor
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
// ENCODER VARIABLES
// ============================================================================
volatile long encoderLeftCount = 0;

void encoderLeftISR() {
  encoderLeftCount++;
}

// ============================================================================
// MOTOR HELPERS
// ============================================================================
void motorForward(uint8_t speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, speed);
}

void motorReverse(uint8_t speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, speed);
}

void motorStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
}

// ============================================================================
// MOTOR TEST STATE MACHINE
// ============================================================================
enum MotorState {
  FWD_RAMP_UP,
  FWD_HOLD,
  FWD_RAMP_DOWN,
  PAUSE_1,
  REV_RAMP_UP,
  REV_HOLD,
  REV_RAMP_DOWN,
  PAUSE_2,
  DONE
};

MotorState motorState    = FWD_RAMP_UP;
int        currentSpeed  = 0;
unsigned long stateTimer = 0;

void updateMotorTest(unsigned long now) {
  if (motorState == DONE) return;

  switch (motorState) {
    case FWD_RAMP_UP:
      if (now - stateTimer >= 30) {
        currentSpeed += 5;
        motorForward(currentSpeed);
        stateTimer = now;
        if (currentSpeed >= 255) { motorState = FWD_HOLD; stateTimer = now; Serial.println(F(">> Full speed forward")); }
      }
      break;

    case FWD_HOLD:
      if (now - stateTimer >= 1000) { motorState = FWD_RAMP_DOWN; stateTimer = now; }
      break;

    case FWD_RAMP_DOWN:
      if (now - stateTimer >= 30) {
        currentSpeed -= 5;
        motorForward(currentSpeed);
        stateTimer = now;
        if (currentSpeed <= 0) { motorStop(); motorState = PAUSE_1; stateTimer = now; Serial.println(F(">> Stop")); }
      }
      break;

    case PAUSE_1:
      if (now - stateTimer >= 1000) { motorState = REV_RAMP_UP; stateTimer = now; Serial.println(F(">> Reverse ramp")); }
      break;

    case REV_RAMP_UP:
      if (now - stateTimer >= 30) {
        currentSpeed += 5;
        motorReverse(currentSpeed);
        stateTimer = now;
        if (currentSpeed >= 255) { motorState = REV_HOLD; stateTimer = now; Serial.println(F(">> Full speed reverse")); }
      }
      break;

    case REV_HOLD:
      if (now - stateTimer >= 1000) { motorState = REV_RAMP_DOWN; stateTimer = now; }
      break;

    case REV_RAMP_DOWN:
      if (now - stateTimer >= 30) {
        currentSpeed -= 5;
        motorReverse(currentSpeed);
        stateTimer = now;
        if (currentSpeed <= 0) { motorStop(); motorState = PAUSE_2; stateTimer = now; Serial.println(F(">> Stop - motor test done")); }
      }
      break;

    case PAUSE_2:
      if (now - stateTimer >= 1000) { motorState = DONE; }
      break;

    case DONE:
      break;
  }
}

// ============================================================================
// ENCODER REPORT
// ============================================================================
unsigned long lastReportTime = 0;
const unsigned long REPORT_INTERVAL = 500;
long lastLeftCount = 0;

void printEncoderReport(unsigned long now) {
  if (now - lastReportTime < REPORT_INTERVAL) return;

  unsigned long deltaTime = now - lastReportTime;

  noInterrupts();
  long leftCount = encoderLeftCount;
  interrupts();

  long leftDelta = leftCount - lastLeftCount;
  float leftSpeed = (leftDelta * 1000.0) / deltaTime;

  Serial.print(F("["));
  Serial.print(now);
  Serial.print(F("ms] Left: "));
  Serial.print(leftCount);
  Serial.print(F(" ("));
  Serial.print(leftSpeed, 1);
  Serial.println(F(" p/s)"));

  lastReportTime = now;
  lastLeftCount  = leftCount;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("\n====================================="));
  Serial.println(F("Encoder + Motor Test - Arduino Nano"));
  Serial.println(F("=====================================\n"));

  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), encoderLeftISR, CHANGE);

  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(STBY, HIGH);

  stateTimer   = millis();
  lastReportTime = millis();

  Serial.println(F(">> Forward ramp"));
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  updateMotorTest(now);
  printEncoderReport(now);
}
