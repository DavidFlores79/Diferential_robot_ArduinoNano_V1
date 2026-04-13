/*
 * Phase 6 — HMC5883L Magnetometer + Yaw Fusion
 *
 * Fuses gyroscope yaw (short-term) with magnetometer heading (long-term)
 * using a complementary filter.
 *
 * Connections (shared I2C bus):
 * - MPU-6050 SDA/SCL → A4/A5 | AD0 → GND (addr 0x68)
 * - HMC5883L SDA/SCL → A4/A5 (addr 0x1E)
 */

#include <Wire.h>

// ============================================================================
// I2C ADDRESSES & REGISTERS
// ============================================================================
const uint8_t MPU_ADDR     = 0x68;
const uint8_t HMC_ADDR     = 0x1E;

// MPU
const uint8_t MPU_PWR      = 0x6B;
const uint8_t GYRO_ZOUT_H  = 0x47;

// HMC5883L
const uint8_t HMC_CONFIG_A = 0x00;
const uint8_t HMC_MODE     = 0x02;
const uint8_t HMC_XOUT_H   = 0x03;

// ============================================================================
// COMPLEMENTARY FILTER COEFFICIENT
// ============================================================================
// Higher alpha = trust gyro more (less magnetic noise, more drift)
// Lower alpha  = trust magnetometer more (less drift, more noise)
const float ALPHA = 0.98;

// ============================================================================
// SCALE
// ============================================================================
const float GYRO_SCALE = 131.0;  // LSB/(°/s) at ±250°/s

// ============================================================================
// MAGNETOMETER CALIBRATION (measure and adjust these)
// ============================================================================
float magOffsetX = 0;
float magOffsetY = 0;

// ============================================================================
// STATE
// ============================================================================
float yawFused  = 0;
float yawGyro   = 0;

unsigned long lastUpdateTime = 0;
unsigned long lastReportTime = 0;
const unsigned long REPORT_INTERVAL = 200;

// ============================================================================
// MPU — read gyro Z only
// ============================================================================
float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  int16_t raw = Wire.read() << 8 | Wire.read();
  return raw / GYRO_SCALE;
}

// ============================================================================
// HMC5883L — read heading
// ============================================================================
float readMagHeading() {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(HMC_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(HMC_ADDR, (uint8_t)6);

  int16_t mx = Wire.read() << 8 | Wire.read();
  int16_t mz = Wire.read() << 8 | Wire.read();  // Z comes before Y in HMC5883L
  int16_t my = Wire.read() << 8 | Wire.read();

  float fx = mx - magOffsetX;
  float fy = my - magOffsetY;

  float heading = atan2(fy, fx) * 180.0 / PI;
  if (heading < 0) heading += 360.0;
  return heading;
}

// ============================================================================
// SETUP SENSORS
// ============================================================================
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);  // wake up
  Wire.endTransmission();
}

void initHMC() {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(HMC_CONFIG_A); Wire.write(0x70);  // 8 samples avg, 15Hz
  Wire.endTransmission();

  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x01); Wire.write(0x20);  // gain = ±1.3Ga
  Wire.endTransmission();

  Wire.beginTransmission(HMC_ADDR);
  Wire.write(HMC_MODE); Wire.write(0x00);  // continuous mode
  Wire.endTransmission();
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("\n====================================="));
  Serial.println(F("Phase 6 — Yaw Fusion"));
  Serial.println(F("=====================================\n"));

  Wire.begin();
  initMPU();
  delay(100);
  initHMC();
  delay(100);

  // Verify both devices
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("ERROR: MPU-6050 not found!")); while(true) { ; }
  }
  Wire.beginTransmission(HMC_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("ERROR: HMC5883L not found!")); while(true) { ; }
  }

  Serial.println(F("Both sensors found."));
  Serial.println(F("NOTE: Set magOffsetX/Y after calibration for accurate heading."));
  Serial.println(F("Format: Gyro yaw | Mag heading | Fused yaw"));
  Serial.println();

  yawFused = readMagHeading();
  lastUpdateTime = millis();
  lastReportTime = millis();
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  float dt = (now - lastUpdateTime) / 1000.0;
  lastUpdateTime = now;

  float gz      = readGyroZ();
  float magHead = readMagHeading();

  // Gyro integration
  yawGyro += gz * dt;

  // Complementary filter
  yawFused = ALPHA * (yawFused + gz * dt) + (1.0 - ALPHA) * magHead;
  if (yawFused < 0)   yawFused += 360.0;
  if (yawFused > 360) yawFused -= 360.0;

  if (now - lastReportTime >= REPORT_INTERVAL) {
    lastReportTime = now;
    Serial.print(F("Gyro: "));
    Serial.print(yawGyro, 1);
    Serial.print(F("° | Mag: "));
    Serial.print(magHead, 1);
    Serial.print(F("° | Fused: "));
    Serial.print(yawFused, 1);
    Serial.println(F("°"));
  }
}
