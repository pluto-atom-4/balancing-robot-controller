#pragma once

// Output and serial helpers for c3_facts (#44). Only this header prints C3FACTS lines.
// Format: C3FACTS <key> <value> <unit>, four tokens, spaces in tokens become '_'. C++11-valid.

#include <Arduino.h>
#include <cstdint>
#include <cstdio>

namespace facts_io {

constexpr uint16_t kModelXl330M077 = 1190;  // verified vs e-Manual (see lib/dxl_units/dxl_units.h)

// Used to NAME a model only. 1200 is UNVERIFIED vs the e-Manual. Never gates safety except 1190.
inline const char* modelName(uint16_t n) {
  return n == 1190 ? "XL330-M077" : (n == 1200 ? "XL330-M288_UNVERIFIED" : "unknown");
}

inline void token(const char* s) {
  for (; *s != 0; ++s) Serial.print(*s == ' ' ? '_' : *s);
}
inline void head(const char* key) {
  Serial.print("C3FACTS ");
  token(key);
  Serial.print(' ');
}
inline void emitStr(const char* key, const char* value, const char* unit) {
  head(key);
  token(value);
  Serial.print(' ');
  Serial.println(unit);
}
inline void emitInt(const char* key, long value, const char* unit) {
  head(key);
  Serial.print(value);
  Serial.print(' ');
  Serial.println(unit);
}
inline void emitFloat(const char* key, float value, const char* unit, int digits) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%.*f", digits, static_cast<double>(value));
  head(key);
  Serial.print(buf);
  Serial.print(' ');
  Serial.println(unit);
}
inline void emitBool(const char* key, bool v) { emitInt(key, v ? 1 : 0, "-"); }

// Human prompt, not a data line.
inline void prompt(const char* text) {
  Serial.print("[facts] ");
  Serial.println(text);
}

// Discard everything pending on USB serial; returns how many bytes were discarded.
inline uint16_t drainSerial() {
  uint16_t n = 0;
  while (Serial.available() > 0) {
    (void)Serial.read();
    ++n;
  }
  return n;
}

}  // namespace facts_io
