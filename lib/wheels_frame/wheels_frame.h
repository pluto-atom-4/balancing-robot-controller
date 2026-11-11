#pragma once

// SIM-ONLY, UNVERIFIED. Must stay C++11-valid (the C3 firmware compile carries -std=gnu++11).
// Frame: WHL2,<seq>,<t_us>,<model>,<rawL>,<rawR>,<state>*HH\n  (model rad/s, raw int32; wheel signs unconfirmed).
// state: 0 run, 1 hold after a fall, 2 bus down; the caller latches both wheels together.
// A non-finite model is refused (format returns 0). Keep in step with freecad-workspace poc/ctrl-wheels/CONTRACT.md S1/S7.

#include <imu_frame.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace wheels_frame {

inline uint8_t checksum(const char* s, size_t n) {
  return imu_frame::checksum(s, n);
}

// Returns the frame length (with '\n', without NUL), or 0 on failure with out[0] = '\0'. cap includes the NUL.
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
