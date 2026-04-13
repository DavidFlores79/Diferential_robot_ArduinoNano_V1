/*
 * Phase 7 — OLED Display + Expressions
 *
 * Displays robot state as simple eye expressions on a 128x64 SSD1306 OLED.
 * Cycles through all states automatically for testing.
 *
 * Library required: Adafruit SSD1306 + Adafruit GFX
 * Install via Arduino Library Manager.
 *
 * Connections:
 * - OLED VCC → 3.3V or 5V (check module)
 * - OLED GND → GND
 * - OLED SDA → A4
 * - OLED SCL → A5
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// OLED CONFIG
// ============================================================================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1   // no reset pin
#define OLED_ADDR    0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================================
// ROBOT STATES
// ============================================================================
enum RobotState { IDLE, MOVING, LIFTED, INTERACTING, ANGRY };
const char* stateNames[] = { "IDLE", "MOVING", "LIFTED", "INTERACTING", "ANGRY" };

RobotState currentState = IDLE;

// ============================================================================
// EXPRESSIONS
// Eye = two circles (left eye, right eye)
// Each state has different eye shape/size/position
// ============================================================================

void drawIdle() {
  // Normal open eyes
  display.fillCircle(40, 32, 14, WHITE);
  display.fillCircle(88, 32, 14, WHITE);
  display.fillCircle(40, 32,  6, BLACK);  // pupil
  display.fillCircle(88, 32,  6, BLACK);
}

void drawMoving() {
  // Eyes looking slightly forward (pupils shifted)
  display.fillCircle(40, 32, 14, WHITE);
  display.fillCircle(88, 32, 14, WHITE);
  display.fillCircle(43, 30,  6, BLACK);
  display.fillCircle(91, 30,  6, BLACK);
}

void drawLifted() {
  // Wide surprised eyes
  display.fillCircle(40, 32, 18, WHITE);
  display.fillCircle(88, 32, 18, WHITE);
  display.fillCircle(40, 32,  5, BLACK);
  display.fillCircle(88, 32,  5, BLACK);
}

void drawInteracting() {
  // Happy — half-circle eyes (squinting)
  display.fillCircle(40, 36, 14, WHITE);
  display.fillRect(26, 18, 28, 18, BLACK);  // mask top half → half-circle
  display.fillCircle(88, 36, 14, WHITE);
  display.fillRect(74, 18, 28, 18, BLACK);
}

void drawAngry() {
  // Angled eyes with frown lines
  display.fillCircle(40, 32, 14, WHITE);
  display.fillCircle(88, 32, 14, WHITE);
  display.fillCircle(40, 34,  6, BLACK);
  display.fillCircle(88, 34,  6, BLACK);
  // Angry eyebrow lines
  display.drawLine(26, 15, 54, 22, WHITE);
  display.drawLine(27, 16, 55, 23, WHITE);
  display.drawLine(74, 22, 102, 15, WHITE);
  display.drawLine(75, 23, 103, 16, WHITE);
}

// ============================================================================
// RENDER STATE
// ============================================================================
void renderState(RobotState state) {
  display.clearDisplay();

  switch (state) {
    case IDLE:         drawIdle();         break;
    case MOVING:       drawMoving();       break;
    case LIFTED:       drawLifted();       break;
    case INTERACTING:  drawInteracting();  break;
    case ANGRY:        drawAngry();        break;
  }

  // State label at bottom
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 56);
  display.print(stateNames[state]);

  display.display();
}

// ============================================================================
// TIMING
// ============================================================================
unsigned long lastStateChange = 0;
const unsigned long STATE_DURATION = 2000;  // show each state for 2s

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("\n====================================="));
  Serial.println(F("Phase 7 — OLED Expressions"));
  Serial.println(F("=====================================\n"));

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("ERROR: SSD1306 not found!"));
    Serial.println(F("Check wiring and I2C address (0x3C or 0x3D)"));
    while (true) { ; }
  }

  display.clearDisplay();
  display.display();
  Serial.println(F("OLED found. Cycling through states..."));

  lastStateChange = millis();
  renderState(currentState);
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  if (now - lastStateChange >= STATE_DURATION) {
    lastStateChange = now;
    currentState = (RobotState)((currentState + 1) % 5);
    Serial.print(F("State: "));
    Serial.println(stateNames[currentState]);
    renderState(currentState);
  }
}
