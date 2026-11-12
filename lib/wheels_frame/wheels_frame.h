#pragma once

// SIM-ONLY, UNVERIFIED. Must stay C++11-valid (C3 firmware compiles with -std=gnu++11). Sync: freecad-workspace poc/ctrl-wheels/CONTRACT.md S1/S7.
// Frame: WHL2,<seq>,<t_us>,<model>,<rawL>,<rawR>,<state>*HH\n  (model rad/s, raw int32; wheel signs unconfirmed).
// state: 0 run, 1 hold after a fall, 2 bus down (caller latches both). HH: uppercase hex XOR of bytes from 'W' up to, not including, '*'.

#include <imu_frame.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace wheels_frame {

inline uint8_t checksum(const char* s, size_t n) {
  return imu_frame::checksum(s, n);
}

// Returns the frame length (with '\n', without NUL), or 0 on failure; cap includes the NUL.
// On failure with cap > 0 and out != nullptr, out[0] = '\0'. Nothing is written at or beyond out[cap].
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
