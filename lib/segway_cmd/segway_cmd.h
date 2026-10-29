#ifndef SEGWAY_CMD_H
#define SEGWAY_CMD_H

// Line parser for an external wheel velocity command. Pure header-only.
// No Arduino, no heap, no dependency on segway_sim.h. C++11-valid.
//
// SIM-ONLY. Intended for programs/ttl_servo under -D TTL_SERVO_EXT_CMD (not
// wired by this file). Units: rad/s. Not hardware-validated.
//
// Line format, one line per '\n' or '\r' (an empty line is ignored):
//   CMD <number>
//   One space, then a plain decimal: optional '+' or '-', digits, at most one
//   '.', digits. At least one digit. No exponent, no hex, no nan/inf, no
//   trailing characters.
// Accepted values are clamped to [-kMaxRadS, +kMaxRadS] (3.0 rad/s, about the
// segway wheel cap). A line longer than kLineMax - 1 characters is rejected
// once at its end; the excess is dropped and the buffer never overflows.
// Any other invalid line counts as rejected. The parser is ready for the next
// line after every line.

#include <stdint.h>

namespace segway_cmd {

constexpr float kMaxRadS = 3.0f;
constexpr unsigned kLineMax = 32;

namespace detail {

// Parses buf[0..n) as "CMD <number>". Returns false for any invalid line and
// writes *value only on success. Digits are accumulated as an integer part and
// a fraction numerator over a power of ten, in double, so that values such as
// 0.5, -1.25 and 2.75 are exact before the cast to float.
inline bool parseLine(const char* buf, unsigned n, float* value) {
  if (n < 5) return false;
  if (buf[0] != 'C' || buf[1] != 'M' || buf[2] != 'D' || buf[3] != ' ') return false;

  unsigned i = 4;
  bool negative = false;
  if (buf[i] == '+' || buf[i] == '-') {
    negative = (buf[i] == '-');
    i++;
  }

  double whole = 0.0;
  double frac = 0.0;
  double scale = 1.0;
  unsigned digits = 0;
  while (i < n && buf[i] >= '0' && buf[i] <= '9') {
    whole = whole * 10.0 + static_cast<double>(buf[i] - '0');
    digits++;
    i++;
  }
  if (i < n && buf[i] == '.') {
    i++;
    while (i < n && buf[i] >= '0' && buf[i] <= '9') {
      scale *= 10.0;
      frac = frac * 10.0 + static_cast<double>(buf[i] - '0');
      digits++;
      i++;
    }
  }
  if (digits == 0 || i != n) return false;

  double v = whole + frac / scale;
  if (negative) v = -v;
  const double hi = static_cast<double>(kMaxRadS);
  if (v > hi) v = hi;
  if (v < -hi) v = -hi;
  *value = static_cast<float>(v);
  return true;
}

}  // namespace detail

struct LineParser {
  char buf_[kLineMax];
  unsigned n_;
  bool over_;
  uint32_t accepted;
  uint32_t rejected;

  LineParser() : n_(0), over_(false), accepted(0), rejected(0) {}

  void reset() {
    n_ = 0;
    over_ = false;
    accepted = 0;
    rejected = 0;
  }

  // Returns true exactly once per complete valid line and writes *out_rad_s
  // then (when out_rad_s is not nullptr). Returns false otherwise, and never
  // writes *out_rad_s.
  bool feed(char c, float* out_rad_s) {
    if (c != '\n' && c != '\r') {
      if (over_) return false;
      if (n_ + 1 >= kLineMax) {  // room for at most kLineMax - 1 characters
        over_ = true;
        return false;
      }
      buf_[n_++] = c;
      return false;
    }

    if (over_) {
      over_ = false;
      n_ = 0;
      rejected++;
      return false;
    }
    if (n_ == 0) return false;  // empty line, for example the LF after CR

    float v = 0.0f;
    const bool ok = detail::parseLine(buf_, n_, &v);
    n_ = 0;
    if (!ok) {
      rejected++;
      return false;
    }
    accepted++;
    if (out_rad_s != nullptr) *out_rad_s = v;
    return true;
  }
};

}  // namespace segway_cmd

#endif  // SEGWAY_CMD_H
