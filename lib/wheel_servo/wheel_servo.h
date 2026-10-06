#pragma once

// Servo-family-neutral wheel interface + WheelPair safety state machine (#31).
// Header-only, float32, no heap, no exceptions. Natively tested.
// The XL330 driver is programs/balance/dxl_wheels.h (bus-bound); a Feetech
// STS3032 driver is PLANNED (later issue, untested on the C3) and only has to
// implement IWheelServo. Nothing here is validated on hardware.

#include <cmath>
#include <cstdint>
#include <dxl_units.h>

namespace wheel_servo {

enum class Fault : uint8_t { None, NoPing, WriteFail, ReadFail, TorqueOffUnconfirmed, Latched };

struct DeviceInfo {
  uint16_t model_number;
  bool have_vel_limit;
  int32_t vel_limit_raw;
};

// ONE servo family driver. Raw units, per-ID.
class IWheelServo {
 public:
  virtual ~IWheelServo() = default;
  virtual bool begin(uint32_t baud) = 0;          // bus init; may be called again with another baud
  virtual bool ping(uint8_t id) = 0;
  virtual bool readInfo(uint8_t id, DeviceInfo& out) = 0;  // false = comm failure
  virtual bool torqueOff(uint8_t id) = 0;         // 3 retries, confirmed by reading torque-enable back
  virtual bool enableVelocityMode(uint8_t id) = 0;  // torque off -> velocity mode -> goal 0 -> torque on
  virtual bool setVelocityRaw(uint8_t id, int32_t raw) = 0;
  // Default: two single writes, BOTH always attempted. A family may override with one packet.
  virtual bool setVelocityRawPair(uint8_t id_a, int32_t raw_a, uint8_t id_b, int32_t raw_b) {
    const bool a = setVelocityRaw(id_a, raw_a);
    const bool b = setVelocityRaw(id_b, raw_b);
    return a && b;
  }
  virtual bool presentVelocityRaw(uint8_t id, int32_t& out) = 0;
  virtual const dxl_units::VelocityUnits& units() const = 0;
  virtual const char* family() const = 0;
  virtual Fault lastFault() const = 0;
};

struct WheelPairConfig {
  uint8_t id_left;
  uint8_t id_right;
  int8_t sign_left;    // +1 or -1
  int8_t sign_right;   // +1 or -1
  float max_rad_s;     // > 0, finite
  uint8_t fail_max;    // >= 1
};

// Two wheels over one IWheelServo. Owns the safety state machine:
// fail_max consecutive write failures, or any begin() failure -> LATCH:
// torque off on BOTH ids, alive()==false, later writes do nothing and return false.
// Only a fresh begin() clears the latch.
class WheelPair {
 public:
  WheelPair(IWheelServo& s, const WheelPairConfig& c)
      : s_(s), c_(c), up_(false), latched_(false), fail_(0), fault_(Fault::None), lim_l_(0), lim_r_(0) {}

  // Order: validate config (no bus traffic) -> s.begin -> ping both -> torque off both (confirmed)
  // -> read limits -> enableVelocityMode both -> goal 0 both. Any failure latches (torque off) and returns false.
  bool begin(uint32_t baud) {
    up_ = false;
    latched_ = false;
    fail_ = 0;
    fault_ = Fault::None;
    if (!std::isfinite(c_.max_rad_s) || !(c_.max_rad_s > 0.0f) || c_.fail_max == 0 ||
        c_.id_left == c_.id_right || (c_.sign_left != 1 && c_.sign_left != -1) ||
        (c_.sign_right != 1 && c_.sign_right != -1)) {
      latched_ = true;
      fault_ = Fault::Latched;
      return false;
    }
    if (!s_.begin(baud)) return fail(Fault::NoPing);
    const bool pl = s_.ping(c_.id_left);
    const bool pr = s_.ping(c_.id_right);
    if (!pl || !pr) return fail(Fault::NoPing);
    if (!offConfirmed()) return fail(Fault::TorqueOffUnconfirmed);
    DeviceInfo il = {};
    DeviceInfo ir = {};
    if (!s_.readInfo(c_.id_left, il) || !s_.readInfo(c_.id_right, ir)) return fail(Fault::ReadFail);
    lim_l_ = limitOf(il);
    lim_r_ = limitOf(ir);
    if (!s_.enableVelocityMode(c_.id_left) || !s_.enableVelocityMode(c_.id_right)) return fail(Fault::WriteFail);
    if (!s_.setVelocityRawPair(c_.id_left, 0, c_.id_right, 0)) return fail(Fault::WriteFail);
    up_ = true;
    return true;
  }

  // NaN/inf -> 0. Clamp to min(max_rad_s, device limit as rad/s). Per-wheel sign. Writes nothing when not up/latched.
  bool writeRadS(float left_rad_s, float right_rad_s) {
    if (!up_ || latched_) return false;
    const int32_t rl = toRaw(left_rad_s, lim_l_, c_.sign_left);
    const int32_t rr = toRaw(right_rad_s, lim_r_, c_.sign_right);
    if (s_.setVelocityRawPair(c_.id_left, rl, c_.id_right, rr)) {
      fail_ = 0;
      fault_ = Fault::None;
      return true;
    }
    fault_ = Fault::WriteFail;
    if (++fail_ >= c_.fail_max) latch(Fault::WriteFail);
    return false;
  }

  void stop() { (void)writeRadS(0.0f, 0.0f); }  // goal 0 both (no-op when latched)

  // Deliberate kill: torque off both (confirmed), then latched (alive()==false, fault()==Latched).
  bool torqueOff() { return latch(Fault::Latched); }

  bool alive() const { return up_ && !latched_; }
  Fault fault() const { return fault_; }

 private:
  bool offConfirmed() {
    const bool a = s_.torqueOff(c_.id_left);
    const bool b = s_.torqueOff(c_.id_right);  // ALWAYS attempt both
    return a && b;
  }
  bool latch(Fault cause) {
    up_ = false;
    if (!latched_) fault_ = cause;
    latched_ = true;
    const bool ok = offConfirmed();
    if (!ok) fault_ = Fault::TorqueOffUnconfirmed;
    return ok;
  }
  bool fail(Fault cause) {
    (void)latch(cause);
    return false;
  }
  int32_t limitOf(const DeviceInfo& i) const {
    return (i.have_vel_limit && i.vel_limit_raw > 0) ? i.vel_limit_raw : s_.units().default_limit_raw;
  }
  int32_t toRaw(float cmd, int32_t limit_raw, int8_t sign) const {
    if (!std::isfinite(cmd)) cmd = 0.0f;
    const float dev = dxl_units::rawToRadS(s_.units(), limit_raw);
    const float cap = (dev < c_.max_rad_s) ? dev : c_.max_rad_s;
    if (cmd > cap) cmd = cap;
    if (cmd < -cap) cmd = -cap;
    return static_cast<int32_t>(sign) * dxl_units::radSToRaw(s_.units(), cmd, limit_raw);
  }

  IWheelServo& s_;
  WheelPairConfig c_;
  bool up_;
  bool latched_;
  uint8_t fail_;
  Fault fault_;
  int32_t lim_l_;
  int32_t lim_r_;
};

}  // namespace wheel_servo
