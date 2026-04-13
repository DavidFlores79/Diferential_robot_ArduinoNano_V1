/*
 * Phase 5 — MPU-6050 IMU Read
 *
 * Reads accelerometer and gyroscope over I2C.
 * No library needed — raw I2C register reads.
 *
 * Connections:
 * - MPU-6050 VCC → 3.3V
 * - MPU-6050 GND → GND
 * - MPU-6050 SDA → A4
 * - MPU-6050 SCL → A5
 * - MPU-6050 AD0 → GND  (I2C address = 0x68)
 */

#include <Wire.h>

// ============================================================================
// MPU-6050 REGISTERS
// ============================================================================
const uint8_t MPU_ADDR       = 0x68;
const uint8_t PWR_MGMT_1     = 0x6B;
const uint8_t ACCEL_XOUT_H   = 0x3B;
const uint8_t GYRO_XOUT_H    = 0x43;

// ============================================================================
// SCALE FACTORS (default ±2g, ±250°/s)
// ============================================================================
const float ACCEL_SCALE = 16384.0;  // LSB/g
const float GYRO_SCALE  = 131.0;   // LSB/(°/s)

// ============================================================================
// TIMING
// ============================================================================
unsigned long lastReportTime = 0;
const unsigned long REPORT_INTERVAL = 200;  // 5Hz report

// ============================================================================
// IMU READ
// ============================================================================
struct IMUData {
  float ax, ay, az;   // g
  float gx, gy, gz;   // deg/s
};

IMUData readIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  IMUData d;
  d.ax = (int16_t)(Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
  d.ay = (int16_t)(Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
  d.az = (int16_t)(Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
  Wire.read(); Wire.read();  // temperature (skip)
  d.gx = (int16_t)(Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
  d.gy = (int16_t)(Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
  d.gz = (int16_t)(Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
  return d;
}

// ============================================================================
// LIFTED DETECTION
// ============================================================================
bool isLifted(IMUData &d) {
  // When lifted, total acceleration drops (free-fall-like)
  float totalAccel = sqrt(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
  return totalAccel < 0.5;  // less than 0.5g total = likely lifted
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("\n====================================="));
  Serial.println(F("Phase 5 — MPU-6050 IMU Read"));
  Serial.println(F("=====================================\n"));

  Wire.begin();

  // Wake up MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);  // clear sleep bit
  Wire.endTransmission();

  delay(100);

  // Verify device is responding
  Wire.beginTransmission(MPU_ADDR);
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    Serial.println(F("ERROR: MPU-6050 not found on I2C bus!"));
    Serial.println(F("Check wiring and AD0 → GND"));
    while (true) { ; }
  }

  Serial.println(F("MPU-6050 found. Reading..."));
  Serial.println(F("Format: ax ay az (g) | gx gy gz (deg/s) | state"));
  Serial.println();

  lastReportTime = millis();
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  if (now - lastReportTime < REPORT_INTERVAL) return;
  lastReportTime = now;

  IMUData d = readIMU();

  Serial.print(F("A: "));
  Serial.print(d.ax, 2); Serial.print(F(" "));
  Serial.print(d.ay, 2); Serial.print(F(" "));
  Serial.print(d.az, 2);
  Serial.print(F(" | G: "));
  Serial.print(d.gx, 1); Serial.print(F(" "));
  Serial.print(d.gy, 1); Serial.print(F(" "));
  Serial.print(d.gz, 1);
  Serial.print(F(" | "));
  Serial.println(isLifted(d) ? F("LIFTED") : F("GROUNDED"));
}
