#ifndef STEPPER_SPEED_H
#define STEPPER_SPEED_H

// Wheel velocity (rad/s) -> step pulses, for the Wokwi stepper_motor program.
// Pure C++11, no Arduino deps, natively tested. SIM-ONLY: nothing here is
// validated on hardware.
//
// StepAccumulator is polled: each pass it is told the demanded step rate and
// the time since the last pass, and returns how many whole steps to emit now.
// The fractional remainder carries over. A per-pass cap bounds the burst; any
// steps above the cap are dropped and counted (no catch-up), so a loop that
// cannot keep up shows up in dropped() instead of being hidden.

#include <math.h>

namespace stepper {

constexpr float kTwoPi = 6.28318531f;

// rad/s -> steps/s. Invalid input (NaN, steps_per_rev <= 0) gives 0.
inline float rad_s_to_steps_s(float wheel_rad_s, float steps_per_rev) {
  if (wheel_rad_s != wheel_rad_s || !(steps_per_rev > 0.0f)) return 0.0f;
  return wheel_rad_s * steps_per_rev / kTwoPi;
}

class StepAccumulator {
 public:
  StepAccumulator()
      : frac_(0.0f), forward_(true), position_(0), emitted_(0), dropped_(0) {}

  // Returns whole steps to emit this pass (<= max_per_pass); sets direction().
  // dt <= 0 or NaN returns 0; dt is clamped to 0.1 s. A direction change
  // discards the fractional remainder so it cannot fire a step backwards.
  unsigned advance(float steps_per_s, float dt_s, unsigned max_per_pass) {
    if (steps_per_s != steps_per_s || !(dt_s > 0.0f)) return 0;
    if (dt_s > 0.1f) dt_s = 0.1f;
    if (steps_per_s == 0.0f) return 0;
    bool fwd = steps_per_s > 0.0f;
    if (fwd != forward_) {
      forward_ = fwd;
      frac_ = 0.0f;
    }
    float mag = fabsf(steps_per_s);
    if (isinf(mag)) mag = 1.0e6f;
    frac_ += mag * dt_s;
    float whole_f = floorf(frac_);
    frac_ -= whole_f;
    unsigned long whole = whole_f > 4.0e9f ? 4000000000UL : (unsigned long)whole_f;
    unsigned n = whole > max_per_pass ? max_per_pass : (unsigned)whole;
    dropped_ += whole - n;
    emitted_ += n;
    position_ += forward_ ? (long)n : -(long)n;
    return n;
  }

  bool forward() const { return forward_; }
  long position() const { return position_; }
  unsigned long emitted() const { return emitted_; }
  unsigned long dropped() const { return dropped_; }

  // Stop: clears the remainder, keeps the totals.
  void hold() { frac_ = 0.0f; }
  void reset() {
    frac_ = 0.0f;
    forward_ = true;
    position_ = 0;
    emitted_ = 0;
    dropped_ = 0;
  }

 private:
  float frac_;
  bool forward_;
  long position_;
  unsigned long emitted_;
  unsigned long dropped_;
};

}  // namespace stepper

#endif
