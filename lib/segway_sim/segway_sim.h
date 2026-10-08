#ifndef SEGWAY_SIM_H
#define SEGWAY_SIM_H

#include <cmath>

// Segway-like inverted pendulum simulator (C++11-valid, no Arduino deps).
// Pure physics: theta_ddot = a*sin(theta) - b*wheel_vel + disturbance
// Integrated with semi-implicit Euler: first update velocity, then angle.
//
// All units in SI: theta in rad, theta_dot in rad/s, wheel_vel in rad/s,
// times in seconds (dt must be positive).

class SegwaySim {
 public:
  // Configuration (sim-only, unvalidated guesses).
  // Gravity moment coefficient: (m*g*L) / (I_frame + I_wheel*gear_ratio^2)
  // Roughly: larger L (taller) or smaller I (lighter) -> more unstable.
  static constexpr float kGravityCoeff = 5.0f;  // Restoring gravity torque coeff.

  // Motor coupling: how much wheel velocity pushes back on the frame.
  // theta_ddot = a*sin(theta) - b*wheel_vel + disturbance
  // Larger b = wheels dampen the frame more.
  static constexpr float kWheelCoupling = 0.8f;

  // Motor first-order lag: wheel_vel' = (target_vel - wheel_vel) / tau_motor.
  // tau_motor in seconds. Smaller tau -> snappier response.
  static constexpr float kMotorTauSec = 0.1f;  // Motor time constant (100 ms)

  // Fall detection: reset if |theta| > this limit (rad).
  static constexpr float kFallThresholdRad = 0.785f;  // ~45 degrees

  // Disturbance coupling: scales accel/gyro input to torque (rad/s^2).
  static constexpr float kDisturbanceGain = 0.5f;  // Smaller = less sensitive to MPU noise.

  // State
  float theta_rad;       // Body angle (0 = upright, radians)
  float theta_dot_rad_s; // Body angular velocity (rad/s)
  float wheel_vel_rad_s; // Wheel velocity (rad/s, after motor lag)
  bool has_fallen;       // True if |theta| exceeded kFallThresholdRad

  // Constructor: start tilted.
  SegwaySim(float init_theta_rad = 0.262f)
      : theta_rad(init_theta_rad), theta_dot_rad_s(0.0f), wheel_vel_rad_s(0.0f),
        has_fallen(false) {}

  // Reset to initial state.
  void reset(float init_theta_rad = 0.262f) {
    theta_rad = init_theta_rad;
    theta_dot_rad_s = 0.0f;
    wheel_vel_rad_s = 0.0f;
    has_fallen = false;
  }

  // Step the simulation.
  // wheel_target_rad_s: target wheel velocity (rad/s), from servo command.
  // disturbance_rad_s2: external torque disturbance from IMU (rad/s^2).
  // dt_sec: time step (seconds). Must be > 0.
  //
  // Physics:
  //   1. Motor lag: wheel_vel' = (target_vel - wheel_vel) / tau
  //   2. Pendulum: theta_ddot = a*sin(theta) - b*wheel_vel + disturbance
  //   3. Semi-implicit Euler:
  //      theta_dot_new = theta_dot + theta_ddot * dt
  //      theta_new = theta + theta_dot_new * dt
  void step(float wheel_target_rad_s, float disturbance_rad_s2, float dt_sec) {
    if (dt_sec <= 0.0f || has_fallen) return;

    // Clamp dt to 1 second (sanity check for very large jitter).
    if (dt_sec > 1.0f) dt_sec = 1.0f;

    // Motor lag: wheel velocity approaches target.
    // wheel_vel' = (target_vel - wheel_vel) / tau_motor
    if (kMotorTauSec > 0.0f) {
      float motor_rate = (wheel_target_rad_s - wheel_vel_rad_s) / kMotorTauSec;
      wheel_vel_rad_s += motor_rate * dt_sec;
    } else {
      wheel_vel_rad_s = wheel_target_rad_s;  // Infinite response if tau = 0.
    }

    // Pendulum physics: theta_ddot = a*sin(theta) - b*wheel_vel + disturbance.
    // Interpretation: gravity tries to tip over (positive feedback).
    // Wheel velocity provides damping (negative feedback, proportional to speed).
    // Disturbance adds external torque.
    float sin_theta = std::sin(theta_rad);
    float gravity_torque = kGravityCoeff * sin_theta;
    float damping = -kWheelCoupling * wheel_vel_rad_s;
    float theta_ddot = gravity_torque + damping + disturbance_rad_s2;

    // Semi-implicit Euler integration.
    theta_dot_rad_s += theta_ddot * dt_sec;
    theta_rad += theta_dot_rad_s * dt_sec;

    // Check fall condition.
    if (std::fabs(theta_rad) > kFallThresholdRad) {
      has_fallen = true;
    }
  }
};

#endif  // SEGWAY_SIM_H
