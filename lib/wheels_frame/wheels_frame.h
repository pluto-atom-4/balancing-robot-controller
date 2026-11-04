#pragma once

// Pure WHL2 two-wheel frame formatter. SIM-ONLY, UNVERIFIED. No hardware, no
// board SDK, no heap. C++11-valid (the C3 firmware compile carries -std=gnu++11).
//
// Frame: WHL2,<seq>,<t_us>,<model>,<rawL>,<rawR>,<state>*HH\n
//   model in rad/s as %.6f (a non-finite model is REFUSED: format returns 0 and
//   writes no frame; a finite value whose printed text is -0 drops its sign).
//   rawL and rawR are the raw goal velocities as passed in, as signed decimals
//   (int32). The firmware will pass the values it writes to each chip (task
//   B12 wires this in). The formatter does no sign logic and no clamping; no
//   sign claim is made here (wheel signs unconfirmed).
//   state: ONE decimal digit 0..2 for both wheels. The formatter does not
//   enforce that both wheels latch together; the caller is responsible.
//   0 = run, 1 = hold after a fall, 2 = bus down.
//   HH = 2 uppercase hex digits, XOR of every byte from 'W' up to but not
//   including '*'.
//
// Sync rules (keep in step with the host side):
//   - The reference is freecad-workspace poc/ctrl-wheels/CONTRACT.md S1 (frame)
//     and S7 (golden lines, asserted by test/test_wheels_frame). poc/ctrl-wheels/
//     is the twin dir of the single-wheel poc/ctrl-wheel/. The Python WHL2
//     parser/formatter is ctrl_wheels_msg.py there (freecad-workspace PR
//     #511). Any format change must be made in both places.
//   - Reuses imu_frame (checksum, %.6f text, -0 rule and append helpers), so
//     there is a single copy of the checksum and of the -0 rule.
//   - The lib/wheel_frame WHL formatter is a separate, unchanged line format.
//
// Pure formatting only. Nothing here is validated on hardware, and this file
// makes no timing or bandwidth claim.

#include <imu_frame.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace wheels_frame {

// XOR of n bytes. n == 0 or s == nullptr gives 0.
inline uint8_t checksum(const char* s, size_t n) {
  return imu_frame::checksum(s, n);
}

// Writes one WHL2 frame into out (capacity cap bytes, NUL included).
// Returns the frame length L, including the final '\n' and excluding the
// NUL; out[L] == '\0' and L + 1 <= cap. Returns 0 on failure: cap too small,
// snprintf error, out == nullptr, cap == 0, state > 2, or a non-finite
// model_rad_s (state and model are checked before any frame text is formatted).
// On failure with cap > 0 and out != nullptr, out[0] = '\0'.
// Nothing is written at or beyond out[cap].
inline size_t format(char* out, size_t cap, uint16_t seq, uint32_t t_us,
                     float model_rad_s, int32_t rawL, int32_t rawR, uint8_t state) {
  if (out == nullptr || cap == 0) return 0;
  if (state > 2) {
    out[0] = '\0';
    return 0;
  }
  if (!std::isfinite(model_rad_s)) {
    out[0] = '\0';
    return 0;
  }

  size_t pos = 0;
  bool ok = imu_frame::detail::put(out, cap, pos, "WHL2,");

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
    ok = std::snprintf(num, sizeof num, "%ld", static_cast<long>(rawL)) >= 0 &&
         imu_frame::detail::put(out, cap, pos, ",") &&
         imu_frame::detail::put(out, cap, pos, num);
  }
  if (ok) {
    ok = std::snprintf(num, sizeof num, "%ld", static_cast<long>(rawR)) >= 0 &&
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

}  // namespace wheels_frame
