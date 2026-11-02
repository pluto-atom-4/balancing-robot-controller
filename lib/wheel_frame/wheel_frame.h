#pragma once

// Pure wheel frame formatter. SIM-ONLY, UNVERIFIED. No hardware, no board
// SDK, no heap.
//
// Frame: WHL,<seq>,<t_us>,<model>,<raw>,<state>*HH\n
//   model in rad/s as %.6f (non-finite becomes 0.000000, printed-text -0
//   drops its sign), raw as signed decimal (servo raw velocity units),
//   state as a decimal digit 0..2.
//   HH = 2 uppercase hex digits, XOR of every byte from 'W' up to but not
//   including '*'.
//   state: 0 = run, 1 = hold after a fall, 2 = bus down.
//
// Sync rules (keep in step with the host side):
//   - freecad-workspace poc/ctrl-wheel/ctrl_wheel_msg.py is the reference
//     format. Any format change must be made in both places.
//   - Reuses imu_frame (checksum, %.6f text, -0 rule and append helpers), so
//     there is a single copy of the checksum and of the -0 rule.
//
// Pure formatting only. Nothing here is validated on hardware, and this file
// makes no timing or bandwidth claim.

#include <imu_frame.h>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace wheel_frame {

// XOR of n bytes. n == 0 or s == nullptr gives 0.
inline uint8_t checksum(const char* s, size_t n) {
  return imu_frame::checksum(s, n);
}

// Writes one frame into out (capacity cap bytes, NUL included).
// Returns the frame length L, including the final '\n' and excluding the
// NUL; out[L] == '\0' and L + 1 <= cap. Returns 0 on failure: cap too small,
// snprintf error, out == nullptr, cap == 0, or state > 2 (rejected before
// any frame text is formatted; only out[0] is cleared). On failure with
// cap > 0 and out != nullptr, out[0] = '\0'. Nothing is written at or beyond
// out[cap].
inline size_t format(char* out, size_t cap, uint16_t seq, uint32_t t_us,
                     float model_rad_s, int32_t raw, uint8_t state) {
  if (out == nullptr || cap == 0) return 0;
  if (state > 2) {
    out[0] = '\0';
    return 0;
  }

  size_t pos = 0;
  bool ok = imu_frame::detail::put(out, cap, pos, "WHL,");

  char num[24];
  if (ok) {
    ok = std::snprintf(num, sizeof num, "%u", static_cast<unsigned>(seq)) >= 0 &&
         imu_frame::detail::put(out, cap, pos, num);
  }
  if (ok) ok = imu_frame::detail::put(out, cap, pos, ",");
  if (ok) {
    ok = std::snprintf(num, sizeof num, "%lu", static_cast<unsigned long>(t_us)) >= 0 &&
         imu_frame::detail::put(out, cap, pos, num);
  }
  if (ok) ok = imu_frame::detail::putFloat(out, cap, pos, model_rad_s);
  if (ok) {
    ok = std::snprintf(num, sizeof num, "%ld", static_cast<long>(raw)) >= 0 &&
         imu_frame::detail::put(out, cap, pos, ",") &&
         imu_frame::detail::put(out, cap, pos, num);
  }
  if (ok) {
    ok = std::snprintf(num, sizeof num, "%u", static_cast<unsigned>(state)) >= 0 &&
         imu_frame::detail::put(out, cap, pos, ",") &&
         imu_frame::detail::put(out, cap, pos, num);
  }

  if (ok) {
    // Checksum covers out[0 .. pos): from 'W' up to, not including, '*'.
    const uint8_t hh = checksum(out, pos);
    static const char kHex[] = "0123456789ABCDEF";
    char tail[4];
    tail[0] = '*';
    tail[1] = kHex[(hh >> 4) & 0x0F];
    tail[2] = kHex[hh & 0x0F];
    tail[3] = '\0';
    ok = imu_frame::detail::put(out, cap, pos, tail) && imu_frame::detail::put(out, cap, pos, "\n");
  }

  if (!ok) {
    out[0] = '\0';
    return 0;
  }
  return pos;
}

}  // namespace wheel_frame
