/*
 * Phase 8 — Vibration Motor PWM Patterns
 *
 * Non-blocking vibration pattern player.
 * Motor driven via NPN transistor (e.g. 2N2222).
 *
 * Connections:
 * - Nano D11 → 1kΩ resistor → transistor base
 * - Transistor collector → vibration motor −
 * - Vibration motor + → 5V
 * - Transistor emitter → GND
 * - Flyback diode across motor terminals (cathode to +)
 */

// ============================================================================
// PIN
// ============================================================================
const uint8_t VIB_PIN = 11;  // PWM pin

// ============================================================================
// PATTERN DEFINITION
// A pattern is a sequence of {pwm, duration_ms} steps. 0 PWM = off.
// ============================================================================
struct Step {
  uint8_t  pwm;
  uint16_t duration;
};

// Pattern: single pulse
const Step PATTERN_PULSE[] = {
  {200, 100}, {0, 100}, {200, 100}, {0, 500}
};

// Pattern: ramp up then fade
const Step PATTERN_RAMP[] = {
  {80,  80}, {130, 80}, {180, 80}, {230, 80}, {255, 200},
  {200, 60}, {150, 60}, {100, 60}, {50,  60}, {0,   400}
};

// Pattern: angry burst
const Step PATTERN_ANGRY[] = {
  {255, 60}, {0, 40}, {255, 60}, {0, 40},
  {255, 60}, {0, 40}, {255, 200}, {0, 600}
};

// Pattern: gentle idle hum
const Step PATTERN_IDLE[] = {
  {90, 400}, {0, 800}
};

// ============================================================================
// PATTERN PLAYER (non-blocking)
// ============================================================================
const Step* activePattern   = nullptr;
uint8_t     patternLength   = 0;
uint8_t     patternStep     = 0;
bool        patternLoop     = false;
unsigned long stepTimer     = 0;

void playPattern(const Step* pattern, uint8_t length, bool loop = false) {
  activePattern = pattern;
  patternLength = length;
  patternStep   = 0;
  patternLoop   = loop;
  stepTimer     = millis();
  analogWrite(VIB_PIN, pattern[0].pwm);
}

void updateVibration(unsigned long now) {
  if (activePattern == nullptr) return;
  if (now - stepTimer < activePattern[patternStep].duration) return;

  patternStep++;
  if (patternStep >= patternLength) {
    if (patternLoop) {
      patternStep = 0;
    } else {
      analogWrite(VIB_PIN, 0);
      activePattern = nullptr;
      return;
    }
  }

  analogWrite(VIB_PIN, activePattern[patternStep].pwm);
  stepTimer = now;
}

bool isPlaying() {
  return activePattern != nullptr;
}

// ============================================================================
// TEST SEQUENCE
// ============================================================================
uint8_t     testPhase     = 0;
unsigned long phaseTimer  = 0;
const unsigned long PHASE_TIMEOUT = 3000;

const char* phaseNames[] = {
  "PULSE", "RAMP", "ANGRY", "IDLE LOOP"
};

void startNextPhase(unsigned long now) {
  switch (testPhase) {
    case 0:
      playPattern(PATTERN_PULSE, sizeof(PATTERN_PULSE) / sizeof(Step));
      break;
    case 1:
      playPattern(PATTERN_RAMP, sizeof(PATTERN_RAMP) / sizeof(Step));
      break;
    case 2:
      playPattern(PATTERN_ANGRY, sizeof(PATTERN_ANGRY) / sizeof(Step));
      break;
    case 3:
      playPattern(PATTERN_IDLE, sizeof(PATTERN_IDLE) / sizeof(Step), true);
      break;
  }
  Serial.print(F("Pattern: "));
  Serial.println(phaseNames[testPhase]);
  phaseTimer = now;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("\n====================================="));
  Serial.println(F("Phase 8 — Vibration Motor Patterns"));
  Serial.println(F("=====================================\n"));

  pinMode(VIB_PIN, OUTPUT);
  analogWrite(VIB_PIN, 0);

  phaseTimer = millis();
  startNextPhase(millis());
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  updateVibration(now);

  // Advance test phase after timeout or when pattern finishes
  bool advance = false;
  if (testPhase < 3 && !isPlaying()) advance = true;           // non-looping done
  if (testPhase == 3 && now - phaseTimer >= PHASE_TIMEOUT) advance = true;  // looping timeout

  if (advance) {
    testPhase = (testPhase + 1) % 4;
    startNextPhase(now);
  }
}
