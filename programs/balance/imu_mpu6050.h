#pragma once

// MPU-6050 IMU backend for the balance firmware. Arduino-only (lives in
// programs/balance, not lib/). Pure math is in lib/imu_math (natively tested).
// Not a hal::Hal subclass: wheels/ticks come later (#31/#32); a HAL adapter will
// wrap read(). Contract: lib/hal_iface/hal_iface.h (rad, rad/s).

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <cmath>
#include <hal_iface.h>
#include <imu_math.h>
#include <tilt.h>

// Sign from tilt-frame pitch to HAL (Webots) pitch. Applied to pitch_rad and,
// together with kGyroPitchSign, to gyro_rad_s[1], so that gyro_rad_s[1] stays equal
// to d(pitch_rad)/dt. UNVERIFIED on hardware (HIL #34 / probe #44).
constexpr float kPitchSign = 1.0f;
// Sign of the gyro Y rate relative to the accel-derived tilt pitch, used inside the
// complementary filter (copy of tilt_servo GYRO_PITCH_SIGN). It is also part of the
// gyro_rad_s[1] output sign: output = kPitchSign * kGyroPitchSign * (gyro_y - bias).
// UNVERIFIED on hardware: if fused pitch moves opposite to accel pitch, flip.
constexpr float kGyroPitchSign = 1.0f;
// Calibration limits. UNVERIFIED guesses; refine from probe #44 data.
constexpr float kMaxBiasRadS = 0.15f;      // |mean| per axis
constexpr float kMaxBiasVar = 4.0e-4f;     // (0.02 rad/s)^2, population variance

class Mpu6050Imu {
 public:
  bool begin(int sda, int scl, uint32_t i2c_hz) {
    began_ = false;
    calibrated_ = false;
    Wire.begin(sda, scl);
    Wire.setClock(i2c_hz);
    if (!mpu_.begin(0x68, &Wire) && !mpu_.begin(0x69, &Wire)) return false;
    Wire.setClock(i2c_hz);  // begin() may reset the bus clock
    mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Wire.setClock(i2c_hz);
    began_ = true;
    return true;
  }

  // Blocking (~samples * 2 ms). Robot must be still. On success the filter is reset.
  bool calibrateGyroBias(uint16_t samples = 500) {
    calibrated_ = false;
    if (!began_ || samples < 2) return false;
    imu_math::BiasAccumulator acc[3];
    for (uint16_t i = 0; i < samples; ++i) {
      sensors_event_t a, g, t;
      if (!mpu_.getEvent(&a, &g, &t)) return false;
      if (!std::isfinite(g.gyro.x) || !std::isfinite(g.gyro.y) || !std::isfinite(g.gyro.z)) return false;
      acc[0].add(g.gyro.x);
      acc[1].add(g.gyro.y);
      acc[2].add(g.gyro.z);
      delay(2);
    }
    bool ok = true;
    for (int k = 0; k < 3; ++k) {
      bias_[k] = acc[k].mean;      // kept even on failure, for diagnostics
      std_[k] = acc[k].stddev();
      if (!imu_math::biasAcceptable(acc[k].mean, acc[k].variance(), kMaxBiasRadS, kMaxBiasVar)) ok = false;
    }
    if (!ok) return false;
    filt_ = tilt::ComplementaryFilter{};  // first read() seeds from accel
    calibrated_ = true;
    return true;
  }

  // false if not calibrated, bad dt, I2C failure or non-finite data. `out` is
  // unspecified on false. dt_s == 0 is accepted (no gyro integration).
  bool read(hal::ImuSample& out, float dt_s) {
    if (!calibrated_) return false;
    if (!std::isfinite(dt_s) || dt_s < 0.0f) return false;
    sensors_event_t a, g, t;
    if (!mpu_.getEvent(&a, &g, &t)) return false;
    if (!std::isfinite(a.acceleration.x) || !std::isfinite(a.acceleration.y) ||
        !std::isfinite(a.acceleration.z) || !std::isfinite(g.gyro.x) ||
        !std::isfinite(g.gyro.y) || !std::isfinite(g.gyro.z)) {
      return false;
    }
    const float accelDeg = tilt::accelPitchDeg(a.acceleration.x, a.acceleration.y, a.acceleration.z);
    const float gyroDegPerSec = kGyroPitchSign * (g.gyro.y - bias_[1]) * tilt::kRadToDeg;
    const float fusedDeg = filt_.update(accelDeg, gyroDegPerSec, dt_s);
    if (!std::isfinite(fusedDeg)) return false;
    out.t_s = static_cast<float>(micros()) * 1e-6f;  // see #32: adapter may overwrite
    out.roll_rad = 0.0f;
    out.pitch_rad = imu_math::pitchRadFromDeg(fusedDeg, kPitchSign);
    out.yaw_rad = 0.0f;
    out.gyro_rad_s[0] = g.gyro.x - bias_[0];
    out.gyro_rad_s[1] = imu_math::gyroRadSOut(g.gyro.y, bias_[1], kPitchSign * kGyroPitchSign);
    out.gyro_rad_s[2] = g.gyro.z - bias_[2];
    return true;
  }

  bool calibrated() const { return calibrated_; }
  float biasRadS(int axis = 1) const { return (axis >= 0 && axis < 3) ? bias_[axis] : 0.0f; }
  float biasStdRadS(int axis = 1) const { return (axis >= 0 && axis < 3) ? std_[axis] : 0.0f; }

 private:
  Adafruit_MPU6050 mpu_;
  tilt::ComplementaryFilter filt_;
  float bias_[3] = {0.0f, 0.0f, 0.0f};
  float std_[3] = {0.0f, 0.0f, 0.0f};
  bool began_ = false;
  bool calibrated_ = false;
};
