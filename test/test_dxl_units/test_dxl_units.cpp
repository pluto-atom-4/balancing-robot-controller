#include <unity.h>
#include <dxl_units.h>
#include <cstdint>
#include <limits>

using dxl_units::kXl330;
using dxl_units::radSToRaw;
using dxl_units::rawToRadS;
using dxl_units::VelocityUnits;

void setUp(void) {}
void tearDown(void) {}

// These tests use kXl330 as family DATA only. They do not prove the constants
// are right (that is the e-Manual citation in dxl_units.h plus the hardware check).
// Unit: 1 raw = 0.229 rpm * 0.104719755 rad/s per rpm = 0.0239808 rad/s.

void test_zero(void) {
  TEST_ASSERT_EQUAL_INT32(0, radSToRaw(kXl330, 0.0f, 1023));
}

// 1.0 / 0.0239808 = 41.70 -> rounds to 42
void test_one_rad_s(void) {
  TEST_ASSERT_EQUAL_INT32(42, radSToRaw(kXl330, 1.0f, 1023));
}

// 0.5 / 0.0239808 = 20.85 -> 21 ; 2.0 / 0.0239808 = 83.40 -> 83
void test_rounding(void) {
  TEST_ASSERT_EQUAL_INT32(21, radSToRaw(kXl330, 0.5f, 1023));
  TEST_ASSERT_EQUAL_INT32(83, radSToRaw(kXl330, 2.0f, 1023));
}

// Sign symmetry: -1.0 -> -42, -0.5 -> -21, -2.0 -> -83.
void test_sign_symmetry(void) {
  TEST_ASSERT_EQUAL_INT32(-42, radSToRaw(kXl330, -1.0f, 1023));
  TEST_ASSERT_EQUAL_INT32(-21, radSToRaw(kXl330, -0.5f, 1023));
  TEST_ASSERT_EQUAL_INT32(-83, radSToRaw(kXl330, -2.0f, 1023));
}

// 100 rad/s -> 4170 raw -> clamp 1023. limit 50: 5.0 rad/s -> 208 -> clamp 50.
// A huge value (3e38) must clamp, not overflow.
void test_clamp(void) {
  TEST_ASSERT_EQUAL_INT32(1023, radSToRaw(kXl330, 100.0f, 1023));
  TEST_ASSERT_EQUAL_INT32(-1023, radSToRaw(kXl330, -100.0f, 1023));
  TEST_ASSERT_EQUAL_INT32(50, radSToRaw(kXl330, 5.0f, 50));
  TEST_ASSERT_EQUAL_INT32(-50, radSToRaw(kXl330, -5.0f, 50));
  TEST_ASSERT_EQUAL_INT32(1023, radSToRaw(kXl330, 3.0e38f, 1023));
  TEST_ASSERT_EQUAL_INT32(-1023, radSToRaw(kXl330, -3.0e38f, 1023));
  // below the limit stays unclamped: 1.0 rad/s with limit 50 -> 42
  TEST_ASSERT_EQUAL_INT32(42, radSToRaw(kXl330, 1.0f, 50));
}

// limit_raw <= 0 -> 0 (fail safe), for both signs of input.
void test_limit_nonpositive(void) {
  TEST_ASSERT_EQUAL_INT32(0, radSToRaw(kXl330, 1.0f, 0));
  TEST_ASSERT_EQUAL_INT32(0, radSToRaw(kXl330, -1.0f, 0));
  TEST_ASSERT_EQUAL_INT32(0, radSToRaw(kXl330, 1.0f, -5));
}

// NaN / +inf / -inf -> 0 (not a clamp to the limit).
void test_nan_inf(void) {
  const float nan_v = std::numeric_limits<float>::quiet_NaN();
  const float inf_v = std::numeric_limits<float>::infinity();
  TEST_ASSERT_EQUAL_INT32(0, radSToRaw(kXl330, nan_v, 1023));
  TEST_ASSERT_EQUAL_INT32(0, radSToRaw(kXl330, inf_v, 1023));
  TEST_ASSERT_EQUAL_INT32(0, radSToRaw(kXl330, -inf_v, 1023));
}

// raw -> rad/s: 42 * 0.0239808 = 1.00719 rad/s ; 0 -> 0.
void test_raw_to_rad_s(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, rawToRadS(kXl330, 0));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.00719f, rawToRadS(kXl330, 42));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -1.00719f, rawToRadS(kXl330, -42));
}

// Round trip raw -> rad/s -> raw within one raw unit.
void test_round_trip(void) {
  const int32_t raws[] = {-1023, -500, -42, -1, 0, 1, 42, 500, 1023};
  for (int32_t raw : raws) {
    const int32_t back = radSToRaw(kXl330, rawToRadS(kXl330, raw), 1023);
    TEST_ASSERT_INT32_WITHIN(1, raw, back);
  }
}

// Synthetic family: functions must not depend on XL330 constants.
// 1 rad/s = 1/0.104719755 = 9.549 rpm -> 10 raw (rpm_per_raw = 1.0)
// 5 rad/s = 47.75 -> 48 ; 20 rad/s = 190.99 -> clamp 100 ; 10 raw = 1.0472 rad/s
void test_synthetic_family(void) {
  const VelocityUnits t = {"test", 1.0f, 100, false};
  TEST_ASSERT_EQUAL_INT32(10, radSToRaw(t, 1.0f, t.default_limit_raw));
  TEST_ASSERT_EQUAL_INT32(48, radSToRaw(t, 5.0f, t.default_limit_raw));
  TEST_ASSERT_EQUAL_INT32(100, radSToRaw(t, 20.0f, t.default_limit_raw));
  TEST_ASSERT_EQUAL_INT32(-100, radSToRaw(t, -20.0f, t.default_limit_raw));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0472f, rawToRadS(t, 10));
}

// Bad family data (rpm_per_raw <= 0) -> 0, never a full-limit command.
void test_bad_units(void) {
  const VelocityUnits bad = {"bad", 0.0f, 100, false};
  TEST_ASSERT_EQUAL_INT32(0, radSToRaw(bad, 1.0f, 100));
}

// kXl330 data as cited from the e-Manual (see dxl_units.h). A change here must come
// with an updated citation.
void test_xl330_data(void) {
  TEST_ASSERT_EQUAL_STRING("XL330", kXl330.family);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.229f, kXl330.rpm_per_raw);
  TEST_ASSERT_EQUAL_INT32(1620, kXl330.default_limit_raw);
  TEST_ASSERT_TRUE(kXl330.verified);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_zero);
  RUN_TEST(test_one_rad_s);
  RUN_TEST(test_rounding);
  RUN_TEST(test_sign_symmetry);
  RUN_TEST(test_clamp);
  RUN_TEST(test_limit_nonpositive);
  RUN_TEST(test_nan_inf);
  RUN_TEST(test_raw_to_rad_s);
  RUN_TEST(test_round_trip);
  RUN_TEST(test_synthetic_family);
  RUN_TEST(test_bad_units);
  RUN_TEST(test_xl330_data);
  return UNITY_END();
}
