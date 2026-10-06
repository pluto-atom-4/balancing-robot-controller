#pragma once

// rad/s <-> servo velocity raw conversion + clamp. Header-only, float32, no heap,
// no Arduino dependency (natively tested). Servo family hidden behind per-family
// DATA (VelocityUnits); the functions are family-independent.
// Not part of the HAL contract: does not touch kContractVersion.
//
// kXl330 values are taken from the ROBOTIS e-Manual for the XL330-M077:
// https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/ (read 2026-10-06)
//  - Goal Velocity (addr 104) and Present Velocity (addr 128): unit 0.229 rev/min.
//  - Velocity Limit (addr 44, 4 bytes, EEPROM area): default 1620, range 0 ~ 2047.
// `verified` means "matches that page". It is NOT a check on the real servo
// (model, limit and unit are confirmed on hardware by probe #44 and checklist #34).
// If the e-Manual changes or the servo model differs: update the data and the citation.
// STS3032 is intentionally NOT defined here (unknown, untested on the C3).

#include <cmath>
#include <cstdint>

namespace dxl_units {

constexpr float kRadSPerRpm = 0.104719755f;  // 2*pi/60 (exact math constant, not servo data)

struct VelocityUnits {
  const char* family;         // "XL330", later "STS3032"
  float rpm_per_raw;          // velocity unit in rpm per raw count
  int32_t default_limit_raw;  // FALLBACK only; callers pass the limit READ FROM THE DEVICE
  bool verified;              // true only with a recorded datasheet citation
};

// XL330-M077 bench servo, values cited above.
constexpr VelocityUnits kXl330 = {"XL330", 0.229f, 1620, /*verified=*/true};

// rad/s -> raw. Round to nearest (half away from zero), clamp to +-limit_raw.
// limit_raw <= 0 -> 0. NaN/inf rad_s -> 0. Bad units (rpm_per_raw not > 0) -> 0.
// Clamp happens in float BEFORE the integer cast, so no int32 overflow.
inline int32_t radSToRaw(const VelocityUnits& u, float rad_s, int32_t limit_raw) {
  if (limit_raw <= 0) return 0;
  if (!std::isfinite(rad_s)) return 0;
  if (!(u.rpm_per_raw > 0.0f)) return 0;
  const float raw_f = rad_s / (kRadSPerRpm * u.rpm_per_raw);
  const float lim_f = static_cast<float>(limit_raw);
  if (raw_f >= lim_f) return limit_raw;
  if (raw_f <= -lim_f) return -limit_raw;
  const float rounded = (raw_f >= 0.0f) ? (raw_f + 0.5f) : (raw_f - 0.5f);
  int32_t r = static_cast<int32_t>(rounded);  // truncation toward zero after +-0.5
  if (r > limit_raw) r = limit_raw;
  if (r < -limit_raw) r = -limit_raw;
  return r;
}

// raw -> rad/s. No clamp (pure unit conversion).
inline float rawToRadS(const VelocityUnits& u, int32_t raw) {
  return static_cast<float>(raw) * u.rpm_per_raw * kRadSPerRpm;
}

}  // namespace dxl_units
