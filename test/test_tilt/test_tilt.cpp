#include <unity.h>
#include <tilt.h>
#include <cmath>

void setUp(void) {}

void tearDown(void) {}

// Test 1: Level accelerometer (0, 0, G) should give ~0 pitch
void test_accelPitchDeg_level(void) {
  const float G = 9.81f;
  float pitch = tilt::accelPitchDeg(0.0f, 0.0f, G);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pitch);
}

// Test 2: 30 degree tilt
void test_accelPitchDeg_30deg(void) {
  const float G = 9.81f;
  const float angle = 30.0f * tilt::kPi / 180.0f;
  float ax = -G * std::sin(angle);
  float az = G * std::cos(angle);
  float pitch = tilt::accelPitchDeg(ax, 0.0f, az);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 30.0f, pitch);
}

// Test 3: Positive ax gives negative pitch
void test_accelPitchDeg_positive_ax_negative_pitch(void) {
  const float G = 9.81f;
  const float angle = 30.0f * tilt::kPi / 180.0f;
  float ax = G * std::sin(angle);
  float az = G * std::cos(angle);
  float pitch = tilt::accelPitchDeg(ax, 0.0f, az);
  TEST_ASSERT_TRUE(pitch < 0.0f);
}

// Test 4: clampf behavior
void test_clampf(void) {
  TEST_ASSERT_EQUAL_FLOAT(5.0f, tilt::clampf(2.0f, 5.0f, 10.0f));  // Below
  TEST_ASSERT_EQUAL_FLOAT(7.0f, tilt::clampf(7.0f, 5.0f, 10.0f));  // Inside
  TEST_ASSERT_EQUAL_FLOAT(10.0f, tilt::clampf(15.0f, 5.0f, 10.0f)); // Above
}

// Test 5: Filter first call seeds angle
void test_complementary_filter_first_call(void) {
  tilt::ComplementaryFilter filt;
  float result = filt.update(12.0f, 0.0f, 0.01f);
  TEST_ASSERT_EQUAL_FLOAT(12.0f, result);
}

// Test 6: Filter converges
void test_complementary_filter_converges(void) {
  tilt::ComplementaryFilter filt;
  filt.update(0.0f, 0.0f, 0.01f);  // Seed at 0
  for (int i = 0; i < 500; i++) {
    filt.update(30.0f, 0.0f, 0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f, filt.angle);
}

// Test 7: Gyro integration
void test_complementary_filter_gyro_integration(void) {
  tilt::ComplementaryFilter filt;
  filt.angle = 0.0f;
  filt.init = true;
  float result = filt.update(0.0f, 10.0f, 0.1f);
  // 0.98 * (0 + 10*0.1) + 0.02*0 = 0.98
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.98f, result);
}

// Test 8: Filter dt <= 0 returns unchanged
void test_complementary_filter_zero_dt(void) {
  tilt::ComplementaryFilter filt;
  filt.angle = 5.0f;
  filt.init = true;
  float result = filt.update(10.0f, 20.0f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, result);
}

// Test 9: degToTicks basic conversions
void test_degToTicks_conversions(void) {
  TEST_ASSERT_EQUAL_INT32(2048, tilt::degToTicks(0.0f, 0, 4095));
  TEST_ASSERT_INT32_WITHIN(1, 2731, tilt::degToTicks(60.0f, 0, 4095));
  TEST_ASSERT_INT32_WITHIN(1, 1365, tilt::degToTicks(-60.0f, 0, 4095));
}

// Test 10: degToTicks clamping
void test_degToTicks_clamping(void) {
  TEST_ASSERT_EQUAL_INT32(4095, tilt::degToTicks(500.0f, 0, 4095));
  TEST_ASSERT_EQUAL_INT32(0, tilt::degToTicks(-500.0f, 0, 4095));
  TEST_ASSERT_EQUAL_INT32(2500, tilt::degToTicks(90.0f, 1500, 2500));
}

// Test 11: 0.5 deg vs 0 deg give different ticks
void test_degToTicks_precision(void) {
  int32_t ticks_0 = tilt::degToTicks(0.0f, 0, 4095);
  int32_t ticks_0_5 = tilt::degToTicks(0.5f, 0, 4095);
  TEST_ASSERT_NOT_EQUAL_INT32(ticks_0, ticks_0_5);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_accelPitchDeg_level);
  RUN_TEST(test_accelPitchDeg_30deg);
  RUN_TEST(test_accelPitchDeg_positive_ax_negative_pitch);
  RUN_TEST(test_clampf);
  RUN_TEST(test_complementary_filter_first_call);
  RUN_TEST(test_complementary_filter_converges);
  RUN_TEST(test_complementary_filter_gyro_integration);
  RUN_TEST(test_complementary_filter_zero_dt);
  RUN_TEST(test_degToTicks_conversions);
  RUN_TEST(test_degToTicks_clamping);
  RUN_TEST(test_degToTicks_precision);
  return UNITY_END();
}
