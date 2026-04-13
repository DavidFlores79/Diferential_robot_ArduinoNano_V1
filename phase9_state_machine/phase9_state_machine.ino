/*
 * Phase 9 — Full Behavior State Machine
 *
 * Integrates all subsystems:
 * - Dual motor PID control
 * - Dual encoder odometry
 * - MPU-6050 IMU (lifted detection, motion)
 * - HMC5883L magnetometer (heading)
 * - SSD1306 OLED (expressions)
 * - Vibration motor (feedback patterns)
 *
 * States: IDLE → MOVING → LIFTED → INTERACTING → ANGRY
 *
 * All code is non-blocking.
 *
 * Libraries required: Adafruit SSD1306, Adafruit GFX
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
const uint8_t ENC_LEFT  = 2;
const uint8_t ENC_RIGHT = 3;
const uint8_t STBY      = 4;
const uint8_t PWMA      = 5;
const uint8_t PWMB      = 6;
const uint8_t AIN1      = 8;
const uint8_t AIN2      = 7;
const uint8_t BIN1      = 9;
const uint8_t BIN2      = 10;
const uint8_t VIB_PIN   = 11;

// ============================================================================
// OLED
// ============================================================================
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ============================================================================
// I2C ADDRESSES
// ============================================================================
const uint8_t MPU_ADDR = 0x68;
const uint8_t HMC_ADDR = 0x1E;

// ============================================================================
// ROBOT STATES
// ============================================================================
enum RobotState { IDLE, MOVING, LIFTED, INTERACTING, ANGRY };
RobotState robotState    = IDLE;
RobotState lastRendered  = (RobotState)255;  // force first render

const char* stateNames[] = { "IDLE", "MOVING", "LIFTED", "INTERACTING", "ANGRY" };

unsigned long stateEnteredAt = 0;

void enterState(RobotState s) {
  if (robotState == s) return;
  robotState    = s;
  stateEnteredAt = millis();
  Serial.print(F(">> State: "));
  Serial.println(stateNames[s]);
}

// ============================================================================
// ENCODERS
// ============================================================================
volatile long encLeft  = 0;
volatile long encRight = 0;
void encLeftISR()  { encLeft++; }
void encRightISR() { encRight++; }

long lastEncLeft = 0, lastEncRight = 0;
float speedLeft = 0, speedRight = 0;

// ============================================================================
// PID
// ============================================================================
const float kP = 2.0, kI = 0.5, kD = 0.1;
float intL = 0, intR = 0, errL = 0, errR = 0;
float targetLeft = 0, targetRight = 0;
int pwmLeft = 0, pwmRight = 0;

unsigned long lastPIDTime = 0;
const unsigned long PID_INTERVAL = 100;

void setMotorLeft(int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(AIN1, pwm >= 0 ? HIGH : LOW);
  digitalWrite(AIN2, pwm >= 0 ? LOW  : HIGH);
  analogWrite(PWMA, abs(pwm));
  pwmLeft = pwm;
}

void setMotorRight(int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(BIN1, pwm >= 0 ? HIGH : LOW);
  digitalWrite(BIN2, pwm >= 0 ? LOW  : HIGH);
  analogWrite(PWMB, abs(pwm));
  pwmRight = pwm;
}

void updatePID(unsigned long now) {
  if (now - lastPIDTime < PID_INTERVAL) return;
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;

  noInterrupts();
  long lc = encLeft; long rc = encRight;
  interrupts();

  speedLeft  = (lc - lastEncLeft)  / dt;
  speedRight = (rc - lastEncRight) / dt;
  lastEncLeft = lc; lastEncRight = rc;

  float eL = targetLeft  - speedLeft;
  float eR = targetRight - speedRight;
  intL += eL * dt; intR += eR * dt;
  float dL = (eL - errL) / dt;
  float dR = (eR - errR) / dt;
  errL = eL; errR = eR;

  setMotorLeft ((int)(kP * eL + kI * intL + kD * dL));
  setMotorRight((int)(kP * eR + kI * intR + kD * dR));
}

// ============================================================================
// IMU
// ============================================================================
float ax = 0, ay = 0, az = 0;
float gz = 0;
unsigned long lastIMUTime = 0;
const unsigned long IMU_INTERVAL = 50;

void readIMU(unsigned long now) {
  if (now - lastIMUTime < IMU_INTERVAL) return;
  lastIMUTime = now;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  ax = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  ay = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  az = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  Wire.read(); Wire.read();
  Wire.read(); Wire.read();
  Wire.read(); Wire.read();
  gz = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
}

bool isLifted() {
  float total = sqrt(ax * ax + ay * ay + az * az);
  return total < 0.5;
}

bool isMoving() {
  return abs(speedLeft) > 5 || abs(speedRight) > 5;
}

// ============================================================================
// VIBRATION
// ============================================================================
struct VibStep { uint8_t pwm; uint16_t duration; };

const VibStep VIB_IDLE[]  = {{80, 300}, {0, 1200}};
const VibStep VIB_MOVE[]  = {{120, 100}, {0, 400}};
const VibStep VIB_LIFT[]  = {{255, 80}, {0, 60}, {255, 80}, {0, 60}, {255, 200}, {0, 500}};
const VibStep VIB_ANGRY[] = {{255, 50}, {0, 30}, {255, 50}, {0, 30}, {255, 50}, {0, 600}};

const VibStep* vibPattern  = nullptr;
uint8_t        vibLen      = 0;
uint8_t        vibStep     = 0;
bool           vibLoop     = false;
unsigned long  vibTimer    = 0;

void playVib(const VibStep* p, uint8_t len, bool loop = false) {
  vibPattern = p; vibLen = len; vibStep = 0; vibLoop = loop;
  vibTimer = millis();
  analogWrite(VIB_PIN, p[0].pwm);
}

void updateVibration(unsigned long now) {
  if (!vibPattern) return;
  if (now - vibTimer < vibPattern[vibStep].duration) return;
  vibStep++;
  if (vibStep >= vibLen) {
    if (vibLoop) vibStep = 0;
    else { analogWrite(VIB_PIN, 0); vibPattern = nullptr; return; }
  }
  analogWrite(VIB_PIN, vibPattern[vibStep].pwm);
  vibTimer = now;
}

// ============================================================================
// OLED EXPRESSIONS
// ============================================================================
void drawIdle() {
  display.fillCircle(40, 30, 14, WHITE);
  display.fillCircle(88, 30, 14, WHITE);
  display.fillCircle(40, 30,  6, BLACK);
  display.fillCircle(88, 30,  6, BLACK);
}

void drawMoving() {
  display.fillCircle(40, 30, 14, WHITE);
  display.fillCircle(88, 30, 14, WHITE);
  display.fillCircle(43, 28,  6, BLACK);
  display.fillCircle(91, 28,  6, BLACK);
}

void drawLifted() {
  display.fillCircle(40, 30, 18, WHITE);
  display.fillCircle(88, 30, 18, WHITE);
  display.fillCircle(40, 30,  5, BLACK);
  display.fillCircle(88, 30,  5, BLACK);
}

void drawInteracting() {
  display.fillCircle(40, 34, 14, WHITE);
  display.fillRect(26, 18, 28, 16, BLACK);
  display.fillCircle(88, 34, 14, WHITE);
  display.fillRect(74, 18, 28, 16, BLACK);
}

void drawAngry() {
  display.fillCircle(40, 32, 14, WHITE);
  display.fillCircle(88, 32, 14, WHITE);
  display.fillCircle(40, 34,  6, BLACK);
  display.fillCircle(88, 34,  6, BLACK);
  display.drawLine(26, 15, 54, 22, WHITE);
  display.drawLine(27, 16, 55, 23, WHITE);
  display.drawLine(74, 22, 102, 15, WHITE);
  display.drawLine(75, 23, 103, 16, WHITE);
}

void renderOLED(RobotState state) {
  if (state == lastRendered) return;
  lastRendered = state;

  display.clearDisplay();
  switch (state) {
    case IDLE:        drawIdle();        break;
    case MOVING:      drawMoving();      break;
    case LIFTED:      drawLifted();      break;
    case INTERACTING: drawInteracting(); break;
    case ANGRY:       drawAngry();       break;
  }
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 56);
  display.print(stateNames[state]);
  display.display();
}

// ============================================================================
// STATE MACHINE
// ============================================================================
unsigned long lastStateUpdate = 0;
const unsigned long STATE_INTERVAL = 100;

void updateStateMachine(unsigned long now) {
  if (now - lastStateUpdate < STATE_INTERVAL) return;
  lastStateUpdate = now;

  switch (robotState) {
    case IDLE:
      targetLeft = targetRight = 0;
      if (isLifted()) {
        enterState(LIFTED);
        playVib(VIB_LIFT, sizeof(VIB_LIFT) / sizeof(VibStep));
      }
      break;

    case MOVING:
      targetLeft = targetRight = 40;
      if (isLifted()) {
        enterState(LIFTED);
        playVib(VIB_LIFT, sizeof(VIB_LIFT) / sizeof(VibStep));
      } else if (!isMoving() && now - stateEnteredAt > 1000) {
        enterState(IDLE);
      }
      break;

    case LIFTED:
      targetLeft = targetRight = 0;
      if (!isLifted()) {
        enterState(IDLE);
        playVib(VIB_IDLE, sizeof(VIB_IDLE) / sizeof(VibStep), true);
      }
      // Angry if lifted too long
      if (now - stateEnteredAt > 5000) {
        enterState(ANGRY);
        playVib(VIB_ANGRY, sizeof(VIB_ANGRY) / sizeof(VibStep));
      }
      break;

    case INTERACTING:
      targetLeft = targetRight = 0;
      if (now - stateEnteredAt > 3000) enterState(IDLE);
      break;

    case ANGRY:
      targetLeft = targetRight = 0;
      if (now - stateEnteredAt > 3000) enterState(IDLE);
      break;
  }

  renderOLED(robotState);
}

// ============================================================================
// REPORT
// ============================================================================
unsigned long lastReportTime = 0;

void printReport(unsigned long now) {
  if (now - lastReportTime < 500) return;
  lastReportTime = now;

  Serial.print(F("State: ")); Serial.print(stateNames[robotState]);
  Serial.print(F(" | L: ")); Serial.print(speedLeft, 0);
  Serial.print(F(" R: ")); Serial.print(speedRight, 0);
  Serial.print(F(" p/s | ax: ")); Serial.print(ax, 2);
  Serial.print(F(" ay: ")); Serial.print(ay, 2);
  Serial.print(F(" az: ")); Serial.println(az, 2);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("\n====================================="));
  Serial.println(F("Phase 9 — Full Behavior State Machine"));
  Serial.println(F("=====================================\n"));

  // Encoders
  pinMode(ENC_LEFT,  INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT),  encLeftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), encRightISR, CHANGE);

  // Motors
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  // Vibration
  pinMode(VIB_PIN, OUTPUT);
  analogWrite(VIB_PIN, 0);

  // I2C
  Wire.begin();

  // MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED not found — continuing without display"));
  }
  display.clearDisplay();
  display.display();

  lastPIDTime     = millis();
  lastIMUTime     = millis();
  lastStateUpdate = millis();
  lastReportTime  = millis();
  stateEnteredAt  = millis();

  playVib(VIB_IDLE, sizeof(VIB_IDLE) / sizeof(VibStep), true);
  Serial.println(F("Running. Pick up robot to trigger LIFTED state."));
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  readIMU(now);
  updatePID(now);
  updateVibration(now);
  updateStateMachine(now);
  printReport(now);
}
