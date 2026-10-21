#ifndef SEGWAY_SIM_H
#define SEGWAY_SIM_H

// Segway-like inverted pendulum + PD loop, for the Wokwi ttl_servo program.
// Pure C++11, no Arduino deps, natively tested. SIM-ONLY: every constant is
// an UNVERIFIED guess, nothing here is validated on hardware.
//
// Plant (SI units, theta = 0 upright, positive theta = leaning forward):
//   theta_ddot = a*sin(theta) - b*wheel_vel + d
// The wheel velocity (after a first-order motor lag) drives the base toward
// the lean, so positive wheel_vel reduces positive theta. Linearised, the PD
// u = Kp*theta + Kd*theta_dot is stable iff (Routh, tau*s^3 + (1)*s^2 +
// (b*Kd - a*tau)*s + (b*Kp - a)): b*Kp > a, b*Kd > a*tau and Kd > tau*Kp.
// Saturation (wheel_max), velocity quantisation and dt jitter are not covered.
// Params must be positive; the helpers below guard the divisions anyway.
// Output: tick_cmd() returns the PD command in rad/s. ttl_servo ignores it and
// sends the lagged plant wheel_vel_rad_s to the Dynamixel chip as raw goal velocity.

#include <math.h>
#include <stdint.h>
#include <dxl_units.h>

namespace segway {

struct Params {
  float gravity_a;          // 1/s^2, unstable pole strength
  float drive_b;            // (rad/s^2) per (rad/s) of wheel velocity
  float motor_tau_s;        // wheel velocity first-order lag
  float fall_rad;           // |theta| above this = fallen
  float wheel_max_rad_s;    // wheel speed cap (rad/s)
  float kp;                 // wheel rad/s per rad of tilt
  float kd;                 // wheel rad/s per rad/s of tilt rate
  float dist_gain;          // rad/s^2 of disturbance per rad/s of gyro Z
  float gyro_clamp_rad_s;   // |gyro Z| limit
  float gyro_deadband_rad_s;

  Params()
      : gravity_a(5.0f), drive_b(2.0f), motor_tau_s(0.1f), fall_rad(0.785f),
        wheel_max_rad_s(3.0f), kp(8.0f), kd(3.0f), dist_gain(1.5f),
        gyro_clamp_rad_s(2.0f), gyro_deadband_rad_s(0.02f) {}
};

inline float clampf(float x, float lo, float hi) {
  if (x != x) return 0.0f;  // NaN -> 0
  return x < lo ? lo : (x > hi ? hi : x);
}

// Gyro Z (rad/s) -> disturbance torque (rad/s^2): deadband, clamp, gain.
inline float disturbance_from_gyro(const Params& p, float gyro_z_rad_s) {
  float g = clampf(gyro_z_rad_s, -p.gyro_clamp_rad_s, p.gyro_clamp_rad_s);
  if (fabsf(g) < p.gyro_deadband_rad_s) g = 0.0f;
  return g * p.dist_gain;
}

// PD wheel velocity command (rad/s); positive when leaning forward.
inline float pd_wheel_cmd(const Params& p, float theta, float theta_dot) {
  return clampf(p.kp * theta + p.kd * theta_dot, -p.wheel_max_rad_s,
                p.wheel_max_rad_s);
}

// Host-side cap in Dynamixel raw units (about 3 rad/s = wheel_max_rad_s at
// 0.229 rpm/unit). The simulated XL330 chip returns 0 for the Velocity Limit
// register, so the cap must be a host constant, never a device read.
constexpr int32_t kWheelCapRaw = 125;

// Wheel command (rad/s) -> Dynamixel goal velocity raw, clamped to +-cap_raw.
// cap_raw <= 0 returns 0 (the frozen-wheel case). NaN returns 0.
inline int32_t wheel_cmd_to_raw(float rad_s, int32_t cap_raw = kWheelCapRaw) {
  return dxl_units::radSToRaw(dxl_units::kXl330, rad_s, cap_raw);
}

class SegwaySim {
 public:
  Params params;
  float theta_rad;
  float theta_dot_rad_s;
  float wheel_vel_rad_s;
  bool has_fallen;

  explicit SegwaySim(float init_theta_rad = 0.262f)
      : params(), theta_rad(init_theta_rad), theta_dot_rad_s(0.0f),
        wheel_vel_rad_s(0.0f), has_fallen(false) {}

  void reset(float init_theta_rad) {
    theta_rad = init_theta_rad;
    theta_dot_rad_s = 0.0f;
    wheel_vel_rad_s = 0.0f;
    has_fallen = false;
  }

  // Physics only. wheel_cmd_rad_s is what the Dynamixel chip was told to do.
  // dt <= 0, NaN, or an already-fallen sim is a no-op; dt is clamped to 0.1 s.
  void step(float wheel_cmd_rad_s, float disturbance_rad_s2, float dt_s) {
    if (!(dt_s > 0.0f) || has_fallen) return;
    if (dt_s > 0.1f) dt_s = 0.1f;
    float alpha = params.motor_tau_s > 0.0f ? dt_s / params.motor_tau_s : 1.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    wheel_vel_rad_s += (wheel_cmd_rad_s - wheel_vel_rad_s) * alpha;
    float ddot = params.gravity_a * sinf(theta_rad) -
                 params.drive_b * wheel_vel_rad_s + disturbance_rad_s2;
    theta_dot_rad_s += ddot * dt_s;  // semi-implicit Euler
    theta_rad += theta_dot_rad_s * dt_s;
    if (fabsf(theta_rad) > params.fall_rad || theta_rad != theta_rad)
      has_fallen = true;
  }

  // One firmware tick for the Dynamixel path: PD command in rad/s (unrounded),
  // then physics step. The caller sends wheel_vel_rad_s (the lagged plant
  // velocity) or this command through wheel_cmd_to_raw.
  float tick_cmd(float gyro_z_rad_s, float dt_s) {
    float cmd = pd_wheel_cmd(params, theta_rad, theta_dot_rad_s);
    step(cmd, disturbance_from_gyro(params, gyro_z_rad_s), dt_s);
    return cmd;
  }
};

// Radians to degrees. Used by the ttl_servo velocity self-test (TTL_SERVO_HORN_SELFTEST) and,
// under -D TTL_SERVO_IMU_FRAMES, to convert the IMU frame gyro from rad/s to deg/s.
constexpr float kRadToDeg = 57.29578f;

}  // namespace segway

#endif  // SEGWAY_SIM_H
