#pragma once

// imu_frame_stub: SIM-ONLY, DISPOSABLE stub for the ttl_servo IMU frame POC (#160/#161).
// Mirrors the API of lib/imu_frame (#156) and freecad-workspace
// poc/esp32-dof/monitor/dof_frame.py. Keep the function bodies in step with those.
// The frame format is IMU,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>*HH\n.
// SWAP RULE: the include of this file does not exist yet. S2 (#162) adds it to
// programs/ttl_servo/main.cpp inside #ifdef TTL_SERVO_IMU_FRAMES. When #156 (PR #168)
// is merged, swap that one line `#include "imu_frame_stub.h"` for `#include <imu_frame.h>`
// and delete this file. The removal issue #166 also deletes the stub if still present
// (as part of #166). `<imu_frame.h>` only resolves after #168 merges.
// Header-only, no heap, no board-framework include. MUST stay C++11-valid
// (firmware compile carries -std=gnu++11).
// Future-state note: no program includes this file yet. S2 (#162) will include it only
// when TTL_SERVO_IMU_FRAMES is defined. Not hardware-validated.
// Non-finite floats (NaN, +-inf) print as 0.000000 here; the Python dof_frame.py raises
// FrameError instead.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cmath>

namespace imu_frame {

// XOR of the n bytes at s. n == 0 or s == nullptr gives 0.
inline uint8_t checksum(const char* s, size_t n) {
  if (s == nullptr) return 0;
  uint8_t x = 0;
  for (size_t i = 0; i < n; ++i) {
    x = static_cast<uint8_t>(x ^ static_cast<uint8_t>(s[i]));
  }
  return x;
}

namespace detail {

// One float as %.6f into tmp (cap bytes). Non-finite becomes 0.0f first.
// A '-' is dropped when every later printed char is '0' or '.'
// (so -0.0f, -1e-9f, -4e-7f print as 0.000000).
// Returns false if the text does not fit in cap.
inline bool formatFloat(char* tmp, size_t cap, float v) {
  const float f = std::isfinite(v) ? v : 0.0f;
  const int n = std::snprintf(tmp, cap, "%.6f", static_cast<double>(f));
  if (n < 0 || static_cast<size_t>(n) >= cap) return false;
  if (tmp[0] == '-') {
    bool all_zero = true;
    for (int i = 1; i < n; ++i) {
      if (tmp[i] != '0' && tmp[i] != '.') {
        all_zero = false;
        break;
      }
    }
    if (all_zero) {
      for (int i = 0; i < n; ++i) {
        // The last pass copies the NUL at tmp[n] to tmp[n-1]; tmp[n] itself is unchanged.
        tmp[i] = tmp[i + 1];
      }
    }
  }
  return true;
}

// Appends ",<float>" at out[pos]. pos < cap is an invariant on entry.
// Returns false if it does not fit.
inline bool appendFloat(char* out, size_t cap, size_t& pos, float v) {
  char tmp[64];
  if (!formatFloat(tmp, sizeof tmp, v)) return false;
  const int n = std::snprintf(out + pos, cap - pos, ",%s", tmp);
  if (n < 0 || static_cast<size_t>(n) >= cap - pos) return false;
  pos += static_cast<size_t>(n);
  return true;
}

}  // namespace detail

// Writes one frame into out (cap bytes), NUL-terminated.
// Returns L = length including the final '\n' (excluding the NUL), requires L + 1 <= cap.
// Returns 0 on failure (cap too small, out or accel_g or gyro_dps null, cap 0,
// any snprintf error).
// On failure with cap > 0 and out != nullptr, out[0] is set to '\0'.
inline size_t format(char* out, size_t cap, uint16_t seq, uint32_t t_us,
                     const float accel_g[3], const float gyro_dps[3]) {
  if (out == nullptr || cap == 0) return 0;
  out[0] = '\0';
  if (accel_g == nullptr || gyro_dps == nullptr) return 0;

  static const char kHex[] = "0123456789ABCDEF";

  // Body: IMU,<seq>,<t_us> then six floats, no trailing separator.
  const int hn = std::snprintf(out, cap, "IMU,%u,%lu",
                               static_cast<unsigned>(seq), static_cast<unsigned long>(t_us));
  if (hn < 0 || static_cast<size_t>(hn) >= cap) {
    out[0] = '\0';
    return 0;
  }
  size_t pos = static_cast<size_t>(hn);
  if (!detail::appendFloat(out, cap, pos, accel_g[0]) ||
      !detail::appendFloat(out, cap, pos, accel_g[1]) ||
      !detail::appendFloat(out, cap, pos, accel_g[2]) ||
      !detail::appendFloat(out, cap, pos, gyro_dps[0]) ||
      !detail::appendFloat(out, cap, pos, gyro_dps[1]) ||
      !detail::appendFloat(out, cap, pos, gyro_dps[2])) {
    out[0] = '\0';
    return 0;
  }

  // Trailer: *HH\n (4 chars) plus the NUL, so pos + 4 must be < cap.
  if (pos + 4 >= cap) {
    out[0] = '\0';
    return 0;
  }
  const uint8_t cs = checksum(out, pos);  // covers 'I' of "IMU," up to but excluding '*'
  out[pos] = '*';
  out[pos + 1] = kHex[(cs >> 4) & 0x0F];
  out[pos + 2] = kHex[cs & 0x0F];
  out[pos + 3] = '\n';
  pos += 4;
  out[pos] = '\0';
  return pos;
}

}  // namespace imu_frame
