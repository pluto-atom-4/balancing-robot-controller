#pragma once

// Pure safety supervisor for the balance loop (#32). Header-only, no heap, no Arduino,
// no exceptions. Decides state transitions and kill conditions from plain inputs; the caller
// (programs/balance/main.cpp) does the IO and calls WheelPair::torqueOff() when latched().
// LATCHED is terminal: only a reset leaves it. First cause wins.
// Nothing here is validated on hardware.

#include <cmath>
#include <cstdint>

namespace balance_supervisor {

enum class State : uint8_t { Startup, Calibrating, WaitingForLean, Running, Latched };
enum class Cause : uint8_t { None, StartupFail, TiltCutoff, ImuFail, NonFinite, WheelFail, SerialKill, Stall };
// Idle: write nothing. Zero: write 0,0. Drive: step controller, then call onCommand().
// Arm: just entered Running; caller resets the controller and writes nothing this tick.
enum class Action : uint8_t { Idle, Zero, Drive, Arm };

struct Config {
  float theta_ref_rad;       // center for gate and cutoff = the controller's own setpoint
  float cutoff_rad;          // > gate_rad
  float gate_rad;            // > 0
  uint8_t gate_hold_ticks;   // >= 1
  uint8_t imu_fail_max;      // >= 1
  uint32_t stall_us;         // > 0
};

struct Tick {
  uint32_t gap_us;   // measured period since the previous tick
  bool first_tick;   // true: gap_us is not a measurement, skip stall check
  bool imu_ok;
  float pitch_rad;   // only used when imu_ok
};

class Supervisor {
 public:
  explicit Supervisor(const Config& c) : c_(c), state_(State::Startup), cause_(Cause::None), hold_(0), imu_fail_(0) {}

  State state() const { return state_; }
  Cause cause() const { return cause_; }
  bool running() const { return state_ == State::Running; }
  bool latched() const { return state_ == State::Latched; }

  void onStartup(bool wheels_ok) {
    if (state_ != State::Startup) return;
    if (!wheels_ok || !valid()) {
      latch(Cause::StartupFail);
      return;
    }
    state_ = State::Calibrating;
  }

  void onCalibration(bool ok) {
    if (state_ != State::Calibrating || !ok) return;
    state_ = State::WaitingForLean;
    hold_ = 0;
    imu_fail_ = 0;
  }

  void onKillRequest(Cause why) { latch(why); }  // no-op when already latched

  Action onTick(const Tick& t) {
    if (state_ != State::WaitingForLean && state_ != State::Running) return Action::Idle;
    const bool run = (state_ == State::Running);
    if (run && !t.first_tick && t.gap_us > c_.stall_us) {  // strict
      latch(Cause::Stall);
      return Action::Idle;
    }
    if (!t.imu_ok) {
      hold_ = 0;
      if (++imu_fail_ >= c_.imu_fail_max) {
        latch(Cause::ImuFail);
        return Action::Idle;
      }
      return run ? Action::Zero : Action::Idle;
    }
    imu_fail_ = 0;
    if (!std::isfinite(t.pitch_rad)) {
      latch(Cause::NonFinite);
      return Action::Idle;
    }
    const float dev = std::fabs(t.pitch_rad - c_.theta_ref_rad);
    if (run) {
      if (dev > c_.cutoff_rad) {  // strict
        latch(Cause::TiltCutoff);
        return Action::Idle;
      }
      return Action::Drive;
    }
    if (dev <= c_.gate_rad) {  // inclusive
      if (++hold_ >= c_.gate_hold_ticks) {
        state_ = State::Running;
        hold_ = 0;
        return Action::Arm;
      }
    } else {
      hold_ = 0;
    }
    return Action::Idle;
  }

  // After Drive + controller step. true = ok to write.
  bool onCommand(float cmd_rad_s, bool ctrl_fault) {
    if (state_ != State::Running) return false;
    if (ctrl_fault || !std::isfinite(cmd_rad_s)) {
      latch(Cause::NonFinite);
      return false;
    }
    return true;
  }

  // Call every tick with WheelPair::alive(). Ignored in Startup (onStartup covers it).
  void onWheels(bool alive) {
    if (alive) return;
    if (state_ == State::Calibrating || state_ == State::WaitingForLean || state_ == State::Running) {
      latch(Cause::WheelFail);
    }
  }

 private:
  void latch(Cause why) {
    if (state_ == State::Latched) return;
    state_ = State::Latched;
    cause_ = why;
  }
  bool valid() const {
    return std::isfinite(c_.theta_ref_rad) && c_.gate_rad > 0.0f && c_.cutoff_rad > c_.gate_rad &&
           c_.gate_hold_ticks >= 1 && c_.imu_fail_max >= 1 && c_.stall_us > 0;
  }

  Config c_;
  State state_;
  Cause cause_;
  uint8_t hold_;
  uint8_t imu_fail_;
};

inline const char* name(State s) {
  switch (s) {
    case State::Startup: return "Startup";
    case State::Calibrating: return "Calibrating";
    case State::WaitingForLean: return "WaitingForLean";
    case State::Running: return "Running";
    case State::Latched: return "Latched";
  }
  return "?";
}

inline const char* name(Cause c) {
  switch (c) {
    case Cause::None: return "none";
    case Cause::StartupFail: return "startup";
    case Cause::TiltCutoff: return "tilt";
    case Cause::ImuFail: return "imu";
    case Cause::NonFinite: return "nonfinite";
    case Cause::WheelFail: return "wheel";
    case Cause::SerialKill: return "serial";
    case Cause::Stall: return "stall";
  }
  return "?";
}

}  // namespace balance_supervisor
