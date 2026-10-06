#include <Arduino.h>

#if defined(BALANCE_CONTROLLER_LQR) || defined(BALANCE_CONTROLLER_PID)
// IMU bring-up (#30) and wheel servo startup (#31). No control loop. Nothing here is validated on hardware.
// The BALANCE_CONTROLLER_* #error guard belongs to #32 (legacy envs still build the stub below).

#include "imu_mpu6050.h"
#include "config.h"
#include "dxl_wheels.h"
#include <wheel_servo.h>

// I2C SDA=D4 (GPIO6), SCL=D5 (GPIO7), as tilt_servo.
static constexpr uint32_t kI2cHz = 400000;
static constexpr uint32_t kLoopUs = 20000;      // 50 Hz sample/filter rate
static constexpr uint32_t kPrintUs = 100000;    // 10 Hz [imu] line
static constexpr uint32_t kRetryMs = 3000;      // calibration retry period

static Mpu6050Imu imu;
static bool imuBegun = false;
static uint32_t lastUs = 0;
static uint32_t lastPrintUs = 0;
static uint32_t lastTryMs = 0;
static hal::ImuSample sample = {};

// Wheel servos (#31). Startup only: no control loop yet (#32). Servo family hidden behind IWheelServo.
static DxlServo servoBus;
static wheel_servo::WheelPair wheels(
    servoBus,
    wheel_servo::WheelPairConfig{kServoIdLeft, kServoIdRight, kLeftWheelSign, kRightWheelSign,
                                 kMaxWheelRadS, kServoFailMax});

// Torque-safe startup. On ANY failure WheelPair has already tried torque off on both ids and is latched;
// the IMU bring-up keeps running read-only. Never retried in loop(): reset to retry.
static void startWheels() {
  if (wheels.begin(kServoBaud)) {
    Serial.printf("[wheel] %s ready baud=%lu ids=%u/%u torque on, goal 0, NO control (UNVERIFIED on hardware)\n",
                  servoBus.family(), static_cast<unsigned long>(kServoBaud),
                  static_cast<unsigned>(kServoIdLeft), static_cast<unsigned>(kServoIdRight));
  } else {
    const bool unconfirmed = (wheels.fault() == wheel_servo::Fault::TorqueOffUnconfirmed);
    Serial.printf("[wheel] startup FAILED fault=%d, torque off, read-only%s\n",
                  static_cast<int>(wheels.fault()), unconfirmed ? " (TORQUE OFF NOT CONFIRMED)" : "");
  }
}

static void tryCalibrate() {
  Serial.println("[imu] calibrating gyro bias, keep robot still");
  if (imu.calibrateGyroBias()) {
    Serial.printf("[imu] bias ok y=%.5f rad/s (UNVERIFIED on hardware)\n", imu.biasRadS(1));
  } else {
    Serial.printf("[imu] bias FAILED y=%.5f std=%.5f rad/s\n", imu.biasRadS(1), imu.biasStdRadS(1));
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[bal] boot (imu bring-up + wheel startup, no control)");
  startWheels();  // servo torque-off is the first servo action, before IMU begin and the blocking calibration
  imuBegun = imu.begin(D4, D5, kI2cHz);
  Serial.println(imuBegun ? "[imu] ready" : "[imu] MPU6050 not found (check SDA=D4 SCL=D5, AD0)");
  if (imuBegun) tryCalibrate();
  lastUs = micros();
  lastPrintUs = lastUs;
  lastTryMs = millis();
}

void loop() {
  const uint32_t now = micros();
  const uint32_t elapsed = now - lastUs;  // unsigned: rollover-safe
  if (elapsed < kLoopUs) return;
  lastUs = now;
  const float dt = static_cast<float>(elapsed) * 1e-6f;

  if (!imuBegun) {
    if (millis() - lastTryMs >= kRetryMs) {
      lastTryMs = millis();
      imuBegun = imu.begin(D4, D5, kI2cHz);
      Serial.println(imuBegun ? "[imu] ready" : "[imu] not found");
      if (imuBegun) tryCalibrate();
    }
    return;
  }
  if (!imu.calibrated()) {
    if (millis() - lastTryMs >= kRetryMs) {
      lastTryMs = millis();
      tryCalibrate();
    }
    return;
  }

  const bool ok = imu.read(sample, dt);
  if (now - lastPrintUs >= kPrintUs) {
    lastPrintUs = now;
    if (ok) {
      Serial.printf("[imu] pitch=%.4f rad (%.2f deg) gyro_y=%.4f rad/s\n",
                    sample.pitch_rad, sample.pitch_rad * tilt::kRadToDeg, sample.gyro_rad_s[1]);
    } else {
      Serial.println("[imu] read failed");
    }
  }
}

#else  // legacy balance-xiao_s3 / balance-esp32dev stub (no Adafruit libs in those envs)

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Balancing Robot Controller initialized");
}

void loop() {
  delay(100);
}

#endif
