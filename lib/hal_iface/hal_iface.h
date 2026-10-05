#pragma once

// HAL contract (C++ mirror of freecad-workspace
// inverted-pendulum-project/07_Simulation/hal/hal.py).
//
// kContractVersion MUST equal HAL_CONTRACT_VERSION in hal.py. Bump in BOTH repos
// on ANY change to units, pitch sign, field order, dt semantics or THETA_REF
// handling. Hand-mirrored; there is no cross-repo CI.
//
// Units: angles in rad, rates in rad/s, times in seconds. NOT degrees.
// lib/tilt works in degrees; convert only inside a backend.
//
// Pitch sign: pitch_rad has the same sign convention as Webots InertialUnit
// getRollPitchYaw()[1]. The MCU backend applies exactly ONE sign constant
// (kPitchSign, balancing-robot-controller#30). That sign is verified only in
// hardware-in-the-loop, not by anything in this header.
//
// gyro_rad_s is rad/s in body axes; index 1 is pitch rate d(pitch)/dt.
//
// IMU/gyro bias calibration is done INSIDE the backend. read_imu() returns
// false until calibration has succeeded; callers never see biased raw data.
//
// Errors: Python raises HalFault. C++ has no exceptions and no heap here:
// read_imu()/read_encoders() return false on a fault, and then `out` is
// unspecified. read_encoders() returns false when the backend has no encoders.
//
// dt semantics (contract v1): wait_next_tick() returns measured dt in seconds.
// The first call after construction returns the nominal control period, not 0.
// A negative return means STOP the loop. Cores treat dt == 0 as P-only /
// no state update (PID; stateless LQR gives its normal output) and dt < 0 as
// output 0 plus a fault flag (cores must pre-check dt < 0).
//
// write_wheel_velocity(): rad/s; callers clamp, a backend may additionally
// clamp for safety. It must not fail on the hot path.
//
// close() exists in Python only and is intentionally absent here.

#include <cstdint>

namespace hal {

constexpr uint32_t kContractVersion = 1;

// Field order mirrors hal.py ImuSample. t_s is on the same clock as Hal::now_s().
struct ImuSample {
  float t_s;
  float roll_rad;
  float pitch_rad;
  float yaw_rad;
  float gyro_rad_s[3];  // index 1 = pitch rate, rad/s
};

// Cumulative wheel angle in rad.
struct EncoderSample {
  float left_rad;
  float right_rad;
};

class Hal {
 public:
  virtual ~Hal() = default;
  virtual bool read_imu(ImuSample& out) = 0;          // false == HalFault
  virtual bool read_encoders(EncoderSample& out) = 0; // false == unsupported or fault
  virtual void write_wheel_velocity(float left_rad_s, float right_rad_s) = 0;
  virtual float now_s() = 0;                          // backend clock, seconds
  virtual float wait_next_tick() = 0;                 // measured dt_s; < 0 == stop
};

}  // namespace hal
