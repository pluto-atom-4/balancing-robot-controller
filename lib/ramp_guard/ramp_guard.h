#pragma once

// ramp_guard: pure decision logic for the c3_facts capped velocity ramp (#44).
// Header-only, no heap. MUST stay C++11-valid (firmware compile carries -std=gnu++11):
// no multi-statement constexpr functions, no default member initializers on brace-initialised structs.
// Natively tested (test/test_ramp_guard). Nothing here is validated on hardware.

#include <cstdint>

namespace ramp_guard {

constexpr int32_t kMaxCapPct = 25;          // hard ceiling on the cap percent (refuse above)
constexpr int32_t kMaxSaneLimitRaw = 2047;  // XL330 Velocity Limit range 0..2047 (e-Manual, see dxl_units.h)
constexpr uint32_t kUpMs = 7000;
constexpr uint32_t kHoldMs = 1000;
constexpr uint32_t kDownMs = 7000;
constexpr uint32_t kTotalMs = kUpMs + kHoldMs + kDownMs;  // 15000
constexpr uint32_t kHardTimeoutMs = 20000;
constexpr uint32_t kGoWindowMs = 60000;

// cap = floor(limit * pct / 100). 0 means REFUSE (limit not in 1..2047, or pct not in 1..kMaxCapPct).
inline int32_t capFromLimit(int32_t limit_raw, int32_t pct) {
  if (limit_raw <= 0 || limit_raw > kMaxSaneLimitRaw) return 0;
  if (pct <= 0 || pct > kMaxCapPct) return 0;
  return (limit_raw * pct) / 100;
}

// Trapezoid goal (raw) at t_ms since ramp start. Integer floor math. 0 outside [0, kTotalMs) or cap <= 0.
inline int32_t goalAt(uint32_t t_ms, int32_t cap) {
  if (cap <= 0) return 0;
  if (t_ms >= kTotalMs) return 0;
  if (t_ms < kUpMs) return (cap * static_cast<int32_t>(t_ms)) / static_cast<int32_t>(kUpMs);
  if (t_ms < kUpMs + kHoldMs) return cap;
  return (cap * static_cast<int32_t>(kTotalMs - t_ms)) / static_cast<int32_t>(kDownMs);
}

inline bool rampDone(uint32_t t_ms) { return t_ms >= kTotalMs; }

// Absolute |present| bound (both signs): cap + cap/4 + 10.
inline int32_t overspeedBound(int32_t cap) { return cap + cap / 4 + 10; }

enum class Abort : uint8_t { None, SerialByte, Timeout, WriteFail, ReadFail, Overspeed };

struct StepInput {
  uint32_t t_ms;
  bool serial_byte;  // any byte typed, or host gone
  bool write_ok;
  bool read_ok;
  int32_t present;
  int32_t cap;
};

// Priority: SerialByte, Timeout (t_ms > kHardTimeoutMs), WriteFail, ReadFail, Overspeed.
inline Abort check(const StepInput& s) {
  if (s.serial_byte) return Abort::SerialByte;
  if (s.t_ms > kHardTimeoutMs) return Abort::Timeout;
  if (!s.write_ok) return Abort::WriteFail;
  if (!s.read_ok) return Abort::ReadFail;
  const int32_t b = overspeedBound(s.cap);
  if (s.present > b || s.present < -b) return Abort::Overspeed;
  return Abort::None;
}

// Wrap-safe elapsed check (millis()-style uint32).
inline bool windowExpired(uint32_t now_ms, uint32_t start_ms, uint32_t window_ms) {
  return static_cast<uint32_t>(now_ms - start_ms) >= window_ms;
}

enum class GoResult : uint8_t { Pending, Go, Reject };

// Accepts exactly the line "GO" (case-sensitive) ended by '\n' or '\r'. Anything else on a line -> Reject.
// A blank line or the '\n' of CRLF -> Pending. Without a newline nothing ever triggers.
struct GoParser {
  char buf[4];
  uint8_t n;
  bool overflow;
  GoParser() : n(0), overflow(false) { buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0; }
  void reset() { n = 0; overflow = false; }
  GoResult feed(char c) {
    if (c == '\n' || c == '\r') {
      if (n == 0 && !overflow) return GoResult::Pending;
      const bool go = (!overflow && n == 2 && buf[0] == 'G' && buf[1] == 'O');
      reset();
      return go ? GoResult::Go : GoResult::Reject;
    }
    if (n < 3) {
      buf[n] = c;
      ++n;
    } else {
      overflow = true;
    }
    return GoResult::Pending;
  }
};

struct Pre {
  bool ping_ok;
  bool model_known;
  bool torque_off_confirmed;
  bool limit_from_device;  // have_vel_limit from the DEVICE, never the fallback
  bool mode_ok;            // mode read as velocity, or mode write explicitly allowed
  bool host_connected;
  bool id_is_bench;
  int32_t limit_raw;
};

inline bool rampAllowed(const Pre& p, int32_t pct) {
  return p.ping_ok && p.model_known && p.torque_off_confirmed && p.limit_from_device && p.mode_ok &&
         p.host_connected && p.id_is_bench && capFromLimit(p.limit_raw, pct) >= 1;
}

}  // namespace ramp_guard
