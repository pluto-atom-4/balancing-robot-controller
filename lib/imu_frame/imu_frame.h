#pragma once

// Pure IMU frame formatter. No hardware, no board SDK, no heap.
//
// Frame: IMU,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>*HH\n
//   accel in g, gyro in deg/s, each float as %.6f.
//   HH = 2 uppercase hex digits, XOR of every byte from 'I' up to but not
//   including '*'.
//
// Sync rules (keep in step with the host side):
//   - freecad-workspace poc/esp32-dof/monitor/dof_frame.py is the reference
//     format. Python RAISES FrameError on NaN/inf; this C++ code substitutes
//     0.0f instead (issue #156 spec). Any other format change must be made in
//     both places.
//   - Printed-text -0 rule: a value whose %.6f text is "-0.000000" (or any
//     "-" followed only by '0' and '.') is printed without the sign, to match
//     Python's "-0.000000" -> "0.000000" mapping.
//   - programs/ttl_servo/imu_frame_stub.h (issue #161) is a disposable mirror
//     of this header, so the later swap is one include line.
//
// Pure formatting only. Nothing here is validated on hardware, and this file
// makes no timing or bandwidth claim.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace imu_frame {

// XOR of n bytes. n == 0 or s == nullptr gives 0.
inline uint8_t checksum(const char* s, size_t n) {
  if (s == nullptr) return 0;
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) {
    x = static_cast<uint8_t>(x ^ static_cast<uint8_t>(s[i]));
  }
  return x;
}

namespace detail {

// Appends str at out[pos]. The NUL is written by snprintf inside the buffer,
// and the append is refused unless the whole string plus NUL fits in cap.
inline bool put(char* out, size_t cap, size_t& pos, const char* str) {
  if (pos >= cap) return false;
  const size_t room = cap - pos;
  const int r = std::snprintf(out + pos, room, "%s", str);
  if (r < 0 || static_cast<size_t>(r) >= room) return false;
  pos += static_cast<size_t>(r);
  return true;
}

// Formats v as %.6f into tmp (64 bytes; FLT_MAX needs about 47). Non-finite
// input becomes 0.0f. Returns the text to print, or nullptr on failure.
inline const char* floatText(float v, char* tmp) {
  if (!std::isfinite(v)) v = 0.0f;
  const int n = std::snprintf(tmp, 64, "%.6f", static_cast<double>(v));
  if (n < 0 || n >= 64) return nullptr;
  if (tmp[0] == '-') {
    bool zero = true;
    for (int i = 1; i < n; i++) {
      if (tmp[i] != '0' && tmp[i] != '.') {
        zero = false;
        break;
      }
    }
    if (zero) return tmp + 1;  // printed-text -0 rule: drop the sign
  }
  return tmp;
}

// Appends ",<value>" for one float.
inline bool putFloat(char* out, size_t cap, size_t& pos, float v) {
  char tmp[64];
  const char* txt = floatText(v, tmp);
  if (txt == nullptr) return false;
  return put(out, cap, pos, ",") && put(out, cap, pos, txt);
}

}  // namespace detail

// Writes one frame into out (capacity cap bytes, NUL included).
// Returns the frame length L, including the final '\n' and excluding the
// NUL; out[L] == '\0' and L + 1 <= cap. Returns 0 on failure: cap too small,
// snprintf error, out == nullptr, cap == 0, or accel_g/gyro_dps == nullptr.
// On failure with cap > 0 and out != nullptr, out[0] = '\0'. Nothing is
// written at or beyond out[cap - 1] + 1, and cap == 0 or out == nullptr
// writes nothing.
inline size_t format(char* out, size_t cap, uint16_t seq, uint32_t t_us,
                     const float accel_g[3], const float gyro_dps[3]) {
  if (out == nullptr || cap == 0) return 0;
  if (accel_g == nullptr || gyro_dps == nullptr) {
    out[0] = '\0';
    return 0;
  }

  size_t pos = 0;
  bool ok = detail::put(out, cap, pos, "IMU,");

  char num[24];
  if (ok) {
    std::snprintf(num, sizeof num, "%u", static_cast<unsigned>(seq));
    ok = detail::put(out, cap, pos, num);
  }
  if (ok) ok = detail::put(out, cap, pos, ",");
  if (ok) {
    std::snprintf(num, sizeof num, "%lu", static_cast<unsigned long>(t_us));
    ok = detail::put(out, cap, pos, num);
  }
  for (int i = 0; ok && i < 3; i++) ok = detail::putFloat(out, cap, pos, accel_g[i]);
  for (int i = 0; ok && i < 3; i++) ok = detail::putFloat(out, cap, pos, gyro_dps[i]);

  if (ok) {
    // Checksum covers out[0 .. pos): from 'I' up to, not including, '*'.
    const uint8_t hh = checksum(out, pos);
    static const char kHex[] = "0123456789ABCDEF";
    char tail[4];
    tail[0] = '*';
    tail[1] = kHex[(hh >> 4) & 0x0F];
    tail[2] = kHex[hh & 0x0F];
    tail[3] = '\0';
    ok = detail::put(out, cap, pos, tail) && detail::put(out, cap, pos, "\n");
  }

  if (!ok) {
    out[0] = '\0';
    return 0;
  }
  return pos;
}

}  // namespace imu_frame
