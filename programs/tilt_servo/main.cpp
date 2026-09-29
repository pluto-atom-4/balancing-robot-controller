#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <tilt.h>

// I2C SDA=D4 (GPIO6), SCL=D5 (GPIO7)
// Dynamixel bus TX=D6 (GPIO21), RX=D7 (GPIO20)
// Crossover: MCU TX -> FE-URT-1 RX, MCU RX -> FE-URT-1 TX
// Serial is USB CDC (build flags) so Serial1 is free for servo bus.

static constexpr uint32_t kI2cHz = 400000;
static constexpr uint32_t kLoopUs = 20000;    // 50 Hz
static constexpr uint32_t kPrintUs = 100000;  // 10 Hz
// +1: gyro Y > 0 raises pitch (see tilt::accelPitchDeg). Flip to -1 if fused moves opposite to accel on hardware.
static constexpr float GYRO_PITCH_SIGN = 1.0f;

static Adafruit_MPU6050 mpu;
static tilt::ComplementaryFilter filt;
static uint32_t lastUs = 0;
static uint32_t lastPrintUs = 0;
static float accelDeg = 0.0f;

static void i2cScan() {
  uint8_t n = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { Serial.printf("[i2c] found 0x%02X\n", addr); ++n; }
  }
  if (n == 0) Serial.println("[i2c] none");
}

static void haltMsg(const char* msg) {
  // No torque enabled yet, so halting is safe. Revisit once servo added.
  for (;;) { Serial.println(msg); delay(2000); }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[tilt] boot");
  Wire.begin(D4, D5);
  Wire.setClock(kI2cHz);
  if (!mpu.begin(0x68, &Wire) && !mpu.begin(0x69, &Wire)) {
    Serial.println("[imu] MPU6050 not found");
    i2cScan();
    haltMsg("[imu] halted: check wiring (SDA=D4 SCL=D5, AD0)");
  }
  Wire.setClock(kI2cHz);  // in case begin() reset the bus clock
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
  lastUs = micros();
  lastPrintUs = lastUs;
  Serial.println("[imu] ready");
}

void loop() {
  const uint32_t now = micros();
  const uint32_t elapsed = now - lastUs;  // unsigned: rollover-safe
  if (elapsed < kLoopUs) return;
  lastUs = now;
  const float dt = static_cast<float>(elapsed) * 1e-6f;

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  accelDeg = tilt::accelPitchDeg(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  const float gyroDegPerSec = GYRO_PITCH_SIGN * g.gyro.y * tilt::kRadToDeg;
  const float fused = filt.update(accelDeg, gyroDegPerSec, dt);

  if (now - lastPrintUs >= kPrintUs) {
    lastPrintUs = now;
    Serial.printf("[imu] accel=%.2f fused=%.2f\n", accelDeg, fused);
  }
}
