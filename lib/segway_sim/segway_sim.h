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
// Saturation (wheel_max), servo quantisation and dt jitter are not covered.
// Params must be positive; the helpers below guard the divisions anyway.
// Servo: continuous rotation, 90 deg = stop, above = forward, below = reverse.

#include <math.h>

namespace segway {

struct Params {
  float gravity_a;          // 1/s^2, unstable pole strength
  float drive_b;            // (rad/s^2) per (rad/s) of wheel velocity
  float motor_tau_s;        // wheel velocity first-order lag
  float fall_rad;           // |theta| above this = fallen
  float wheel_max_rad_s;    // wheel speed at servo 0 / 180 deg
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

// Wheel command (rad/s) -> whole-degree servo angle, 90 = stop.
inline int wheel_to_servo_deg(const Params& p, float wheel_rad_s) {
  if (!(p.wheel_max_rad_s > 0.0f)) return 90;
  float w = clampf(wheel_rad_s, -p.wheel_max_rad_s, p.wheel_max_rad_s);
  float deg = 90.0f + (w / p.wheel_max_rad_s) * 90.0f;
  return (int)floorf(deg + 0.5f);
}

inline float servo_deg_to_wheel(const Params& p, int servo_deg) {
  int d = servo_deg < 0 ? 0 : (servo_deg > 180 ? 180 : servo_deg);
  return ((float)(d - 90) / 90.0f) * p.wheel_max_rad_s;
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

  // Physics only. wheel_cmd_rad_s is what the servo was told to do.
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

  // One firmware tick: PD -> servo degrees -> (rounded) wheel command ->
  // physics. Returns the servo angle to write. Shared by firmware and tests.
  int tick(float gyro_z_rad_s, float dt_s) {
    int servo = wheel_to_servo_deg(
        params, pd_wheel_cmd(params, theta_rad, theta_dot_rad_s));
    step(servo_deg_to_wheel(params, servo),
         disturbance_from_gyro(params, gyro_z_rad_s), dt_s);
    return servo;
  }
};

// ---- Horn driver (Wokwi visual mode) -------------------------------------
// wokwi-servo is positional (0..180 deg); it cannot spin on pulse width. To
// show wheel speed, the wheel velocity is integrated into a horn angle that
// wraps at 180 deg. With horn="double" in diagram.json the horn looks the same
// at 0 and 180, so the wrap is (assumed to be) visually seamless. This puts a
// POSITION on the servo pin: never use it with a real continuous-rotation
// servo (build with -D TTL_SERVO_SPEED_CMD for that).

constexpr float kRadToDeg = 57.29578f;

class HornDriver {
 public:
  float angle_deg;   // [0, 180)
  float direction;   // +1: forward wheel = increasing angle, -1: reverse

  explicit HornDriver(float dir = 1.0f) : angle_deg(0.0f), direction(dir) {}

  void reset(float angle = 0.0f) { angle_deg = wrap(angle); }

  // Physical 1:1: horn deg/s = wheel rad/s * 57.2958. NaN/inf wheel and
  // dt <= 0 are no-ops; dt is clamped to 0.1 s like SegwaySim::step.
  void advance(float wheel_vel_rad_s, float dt_s) {
    if (!(dt_s > 0.0f)) return;
    if (!(fabsf(wheel_vel_rad_s) <= 1.0e6f)) return;  // NaN or inf
    if (dt_s > 0.1f) dt_s = 0.1f;
    angle_deg = wrap(angle_deg + direction * wheel_vel_rad_s * kRadToDeg * dt_s);
  }

  static float wrap(float a) {
    if (!(fabsf(a) <= 1.0e6f)) return 0.0f;
    a = fmodf(a, 180.0f);
    if (a < 0.0f) a += 180.0f;
    if (a >= 180.0f) a = 0.0f;  // a + 180 can round up to exactly 180
    return a;
  }
};

// Horn angle (0..180) -> pulse width in microseconds, rounded and clamped.
inline int angle_to_us(float angle_deg, int min_us, int max_us) {
  float a = clampf(angle_deg, 0.0f, 180.0f);
  float us = (float)min_us + (a / 180.0f) * (float)(max_us - min_us);
  return (int)floorf(us + 0.5f);
}

inline float us_to_angle(int us, int min_us, int max_us) {
  if (max_us <= min_us) return 0.0f;
  return clampf(((float)(us - min_us) / (float)(max_us - min_us)) * 180.0f,
                0.0f, 180.0f);
}

}  // namespace segway

#endif  // SEGWAY_SIM_H
