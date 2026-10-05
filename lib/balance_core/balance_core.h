#pragma once

// Header-only float32 PID + LQR balance core. No heap, no exceptions.
// Port of freecad-workspace inverted-pendulum-project/07_Simulation/hal/control_core.py
// (PidBalance / LqrBalance). Gains are constructor args: this header does NOT
// include any generated gains header. Hand-computed tests only here; parity
// against Python vectors is balancing-robot-controller#29.
// Units: pitch rad, gyro rad/s, dt seconds, output rad/s (wheel velocity cmd).
// Sign conventions are UNVERIFIED in simulation and on hardware (see #34).
// dt semantics (hal contract v1): dt < 0 -> {0, 0, fault=true}, no state change.
// dt == 0 -> PID is P-only with no state update; LQR is stateless and normal.

#include <hal_iface.h>

namespace balance {

struct ControlOutput {
  float cmd_rad_s;  // clamped to +-out_limit
  float raw_cmd;    // unclamped (PID dt>0: p+i+d; dt==0: kp*error)
  bool fault;       // true when dt < 0 (or not a number)
};

namespace detail {
// Non-positive limit becomes 0 (output always 0). Python raises ValueError.
inline float sanitize_limit(float limit) { return limit > 0.0f ? limit : 0.0f; }
inline float clampf(float v, float limit) {
  return v > limit ? limit : (v < -limit ? -limit : v);
}
}  // namespace detail

class PidBalance {
 public:
  PidBalance(float kp, float ki, float kd, float out_limit)
      : kp_(kp), ki_(ki), kd_(kd), limit_(detail::sanitize_limit(out_limit)),
        integral_(0.0f), prev_error_(0.0f), has_prev_(false) {}

  ControlOutput step(const hal::ImuSample& imu, float dt_s) {
    // sign UNVERIFIED, do not change: error = -pitch (same as Python).
    const float error = -imu.pitch_rad;
    if (!(dt_s >= 0.0f)) {  // dt < 0 or NaN: fault, state untouched
      return ControlOutput{0.0f, 0.0f, true};
    }
    if (dt_s == 0.0f) {  // P-only, clamped, NO state update
      const float raw0 = kp_ * error;
      return ControlOutput{detail::clampf(raw0, limit_), raw0, false};
    }
    const float p = kp_ * error;
    integral_ += error * dt_s;
    integral_ = detail::clampf(integral_, limit_);  // integral state bound = +-out_limit
    const float i = ki_ * integral_;
    const float d = has_prev_ ? (kd_ * (error - prev_error_) / dt_s) : 0.0f;
    const float raw = p + i + d;
    prev_error_ = error;
    has_prev_ = true;
    return ControlOutput{detail::clampf(raw, limit_), raw, false};
  }

  void reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    has_prev_ = false;
  }

 private:
  float kp_, ki_, kd_, limit_;
  float integral_;
  float prev_error_;
  bool has_prev_;
};

class LqrBalance {
 public:
  LqrBalance(float k_theta, float k_theta_dot, float theta_ref_rad, float out_limit)
      : k_theta_(k_theta), k_theta_dot_(k_theta_dot), theta_ref_(theta_ref_rad),
        limit_(detail::sanitize_limit(out_limit)) {}

  // dt is not used in the math; it is only checked for the fault case.
  ControlOutput step(const hal::ImuSample& imu, float dt_s) const {
    if (!(dt_s >= 0.0f)) {
      return ControlOutput{0.0f, 0.0f, true};
    }
    const float theta = imu.pitch_rad - theta_ref_;
    const float theta_dot = imu.gyro_rad_s[1];
    const float raw = -(k_theta_ * theta + k_theta_dot_ * theta_dot);
    return ControlOutput{detail::clampf(raw, limit_), raw, false};
  }

  void reset() {}  // stateless

 private:
  float k_theta_, k_theta_dot_, theta_ref_, limit_;
};

}  // namespace balance
