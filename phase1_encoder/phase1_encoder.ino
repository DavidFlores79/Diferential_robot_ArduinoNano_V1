/*
 * Phase 1 — LM393 Encoder Interrupt Test
 *
 * Validates interrupt-based pulse counting from optical encoder.
 * No motor driver needed — spin the wheel by hand.
 *
 * Connections:
 * - LM393 VCC → 5V
 * - LM393 GND → GND
 * - LM393 D0  → D2 (INT0)
 * - 104 capacitor between D0 and GND (noise filter)
 *
 * Pass criteria:
 * - Count increments cleanly when wheel spins by hand
 * - No phantom counts while wheel is at rest
 */

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
const uint8_t ENCODER_PIN = 2;  // INT0

// ============================================================================
// ENCODER VARIABLES
// ============================================================================
volatile long encoderCount = 0;

void encoderISR() {
  encoderCount++;
}

// ============================================================================
// TIMING
// ============================================================================
unsigned long lastReportTime = 0;
const unsigned long REPORT_INTERVAL = 500;
long lastCount = 0;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("\n================================="));
  Serial.println(F("Phase 1 — LM393 Encoder Test"));
  Serial.println(F("================================="));
  Serial.println(F("Spin the wheel by hand to test."));
  Serial.println(F("Format: [Time(ms)] Count (pulses/s)"));
  Serial.println();

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);

  lastReportTime = millis();
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  if (now - lastReportTime < REPORT_INTERVAL) return;

  unsigned long dt = now - lastReportTime;
  lastReportTime = now;

  noInterrupts();
  long count = encoderCount;
  interrupts();

  long delta = count - lastCount;
  lastCount  = count;
  float speed = (delta * 1000.0) / dt;

  Serial.print(F("["));
  Serial.print(now);
  Serial.print(F("ms] Count: "));
  Serial.print(count);
  Serial.print(F("  Speed: "));
  Serial.print(speed, 1);
  Serial.println(F(" p/s"));
}
