#pragma once

#include <cmath>
#include <cstdint>

namespace tilt {

// Mathematical constants
constexpr float kPi = 3.14159265358979f;
constexpr float kRadToDeg = 180.0f / kPi;

// Sign convention: positive pitch = -Ax (accel X axis pointing down gives negative pitch).
inline float accelPitchDeg(float ax, float ay, float az) {
  return std::atan2(-ax, std::sqrt(ay * ay + az * az)) * kRadToDeg;
}

// Clamp value v to range [lo, hi]
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Complementary filter for angle estimation combining accelerometer and gyro
struct ComplementaryFilter {
  float alpha = 0.98f;  // Filter weight (0.98 = 98% gyro, 2% accel)
  float angle = 0.0f;   // Current angle estimate in degrees
  bool init = false;    // Initialization flag

  float update(float accelDeg, float gyroDegPerSec, float dtSec) {
    if (!init) {
      angle = accelDeg;
      init = true;
      return angle;
    }
    if (dtSec <= 0.0f) return angle;
    angle = alpha * (angle + gyroDegPerSec * dtSec) + (1.0f - alpha) * accelDeg;
    return angle;
  }
};

// Convert degrees to servo position ticks
// Clamps result to [minTicks, maxTicks]
inline int32_t degToTicks(float deg, int32_t minTicks, int32_t maxTicks,
                          float centerTicks = 2048.0f,
                          float ticksPerDeg = 4096.0f / 360.0f) {
  const int32_t t = static_cast<int32_t>(std::lround(centerTicks + deg * ticksPerDeg));
  return t < minTicks ? minTicks : (t > maxTicks ? maxTicks : t);
}

}  // namespace tilt
