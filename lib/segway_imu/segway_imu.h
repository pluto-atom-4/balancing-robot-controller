#pragma once

// Synthetic IMU state for the Segway sim (epic #184, S1 #186). Pure header-only.
// No Arduino, no heap, no dependency on segway_sim.h. C++11-valid.
//
// SIM-ONLY synthetic stream for the epic #184 ttl_servo DOF flag. Gravity
// rotation only: no wheel-acceleration term, deterministic, no noise, no bias.
// Signs derived in issue #185 (S0 sign table) and UNVERIFIED on hardware.
//
// Convention (theta = 0 upright, theta > 0 = lean forward, nose = +X):
//   ax = -sin(theta) g      ay = 0      az = +cos(theta) g   (flat: +1 g on Z)
//   gx = 0                  gy = theta_dot * 57.29578 deg/s  gz = 0 (yaw, never a pitch input)
//
// Input sanitising: a non-finite theta or theta_dot is treated as 0, so a NaN
// theta gives the rest vector (0, 0, 1). Output clamps: gyro to +-250 deg/s
// (ttl_servo MPU range), accel to +-2 g. Outputs are normalised so that no
// component is -0.0f. The arrays feed imu_frame::format directly.

#include <cmath>

namespace segway_imu {

constexpr float kRadToDeg = 57.29578f;
constexpr float kGyroMaxDps = 250.0f;
constexpr float kAccelMaxG = 2.0f;

namespace detail {

inline float finiteOrZero(float v) { return std::isfinite(v) ? v : 0.0f; }

inline float clampTo(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// IEEE round-to-nearest: -0.0f + 0.0f == +0.0f, every other value is unchanged.
inline float noNegZero(float v) { return v + 0.0f; }

}  // namespace detail

// theta_rad: lean angle (rad), theta_dot_rad_s: lean rate (rad/s).
// accel_g[3] gets (ax, ay, az) in g; gyro_dps[3] gets (gx, gy, gz) in deg/s.
inline void from_state(float theta_rad, float theta_dot_rad_s, float accel_g[3], float gyro_dps[3]) {
  if (accel_g == nullptr || gyro_dps == nullptr) return;

  const float theta = detail::finiteOrZero(theta_rad);
  const float theta_dot = detail::finiteOrZero(theta_dot_rad_s);

  const float ax = detail::clampTo(-std::sin(theta), -kAccelMaxG, kAccelMaxG);
  const float az = detail::clampTo(std::cos(theta), -kAccelMaxG, kAccelMaxG);
  accel_g[0] = detail::noNegZero(ax);
  accel_g[1] = 0.0f;
  accel_g[2] = detail::noNegZero(az);

  const float gy = detail::clampTo(theta_dot * kRadToDeg, -kGyroMaxDps, kGyroMaxDps);
  gyro_dps[0] = 0.0f;
  gyro_dps[1] = detail::noNegZero(gy);
  gyro_dps[2] = 0.0f;  // yaw: never driven by the pitch state
}

}  // namespace segway_imu
