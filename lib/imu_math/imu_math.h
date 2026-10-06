#pragma once

// Pure IMU math for the MPU-6050 backend. No Arduino includes, no heap, natively
// testable (test/test_imu_math). Degrees->radians conversion lives here and in
// programs/balance/imu_mpu6050.h only. Signs are passed in by the caller.

#include <cmath>
#include <cstdint>
#include <tilt.h>

namespace imu_math {

constexpr float kDegToRad = tilt::kPi / 180.0f;

// Filter output (degrees, tilt frame) -> HAL pitch in rad. sign = kPitchSign.
inline float pitchRadFromDeg(float deg, float sign) {
  return sign * deg * kDegToRad;
}

// Bias-subtracted gyro rate (rad/s) times sign.
inline float gyroRadSOut(float raw_rad_s, float bias_rad_s, float sign) {
  return sign * (raw_rad_s - bias_rad_s);
}

// Welford running mean / population variance. Float, no heap.
struct BiasAccumulator {
  uint32_t n = 0;
  float mean = 0.0f;
  float m2 = 0.0f;

  void reset() { n = 0; mean = 0.0f; m2 = 0.0f; }

  void add(float x) {
    ++n;
    const float delta = x - mean;
    mean += delta / static_cast<float>(n);
    m2 += delta * (x - mean);
  }

  float variance() const { return n > 0 ? m2 / static_cast<float>(n) : 0.0f; }
  float stddev() const { return std::sqrt(variance()); }
};

// True only if mean and variance are finite and within limits.
inline bool biasAcceptable(float mean, float variance, float maxAbsMean, float maxVariance) {
  if (!std::isfinite(mean) || !std::isfinite(variance)) return false;
  return std::fabs(mean) <= maxAbsMean && variance <= maxVariance;
}

}  // namespace imu_math
