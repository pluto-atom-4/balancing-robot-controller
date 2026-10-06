#include <unity.h>
#include <imu_math.h>
#include <tilt.h>
#include <cmath>

void setUp(void) {}
void tearDown(void) {}

void test_deg_to_rad_constant(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, tilt::kPi, 180.0f * imu_math::kDegToRad);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5707964f, 90.0f * imu_math::kDegToRad);
}

void test_pitch_rad_sign(void) {
  // 30 deg = 0.5235988 rad
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5235988f, imu_math::pitchRadFromDeg(30.0f, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.5235988f, imu_math::pitchRadFromDeg(30.0f, -1.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, imu_math::pitchRadFromDeg(0.0f, 1.0f));
}

void test_gyro_out_bias_and_sign(void) {
  // 0.30 - 0.05 = 0.25
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.25f, imu_math::gyroRadSOut(0.30f, 0.05f, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.25f, imu_math::gyroRadSOut(0.30f, 0.05f, -1.0f));
}

// Welford on 1,2,3,4: mean 2.5, m2 = 5, population variance 1.25, stddev sqrt(1.25) = 1.118034
void test_accumulator_known_values(void) {
  imu_math::BiasAccumulator a;
  const float xs[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  for (int i = 0; i < 4; ++i) a.add(xs[i]);
  TEST_ASSERT_EQUAL_UINT32(4, a.n);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.5f, a.mean);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.25f, a.variance());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.118034f, a.stddev());
}

void test_accumulator_constant_and_empty(void) {
  imu_math::BiasAccumulator a;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, a.variance());
  for (int i = 0; i < 10; ++i) a.add(0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, a.mean);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, a.variance());
  a.reset();
  TEST_ASSERT_EQUAL_UINT32(0, a.n);
}

void test_bias_acceptable(void) {
  TEST_ASSERT_TRUE(imu_math::biasAcceptable(0.05f, 1e-5f, 0.15f, 4e-4f));
  TEST_ASSERT_TRUE(imu_math::biasAcceptable(-0.05f, 1e-5f, 0.15f, 4e-4f));
  TEST_ASSERT_FALSE(imu_math::biasAcceptable(0.20f, 1e-5f, 0.15f, 4e-4f));
  TEST_ASSERT_FALSE(imu_math::biasAcceptable(-0.20f, 1e-5f, 0.15f, 4e-4f));
  TEST_ASSERT_FALSE(imu_math::biasAcceptable(0.05f, 1e-3f, 0.15f, 4e-4f));
  TEST_ASSERT_FALSE(imu_math::biasAcceptable(NAN, 1e-5f, 0.15f, 4e-4f));
  TEST_ASSERT_FALSE(imu_math::biasAcceptable(0.05f, INFINITY, 0.15f, 4e-4f));
}

void test_pitch_pipeline_with_tilt(void) {
  // accel pitch of a 30 deg lean (tilt convention) -> rad, then sign flip
  const float G = 9.81f;
  const float ang = 30.0f * tilt::kPi / 180.0f;
  const float deg = tilt::accelPitchDeg(-G * std::sin(ang), 0.0f, G * std::cos(ang));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5236f, imu_math::pitchRadFromDeg(deg, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.5236f, imu_math::pitchRadFromDeg(deg, -1.0f));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_deg_to_rad_constant);
  RUN_TEST(test_pitch_rad_sign);
  RUN_TEST(test_gyro_out_bias_and_sign);
  RUN_TEST(test_accumulator_known_values);
  RUN_TEST(test_accumulator_constant_and_empty);
  RUN_TEST(test_bias_acceptable);
  RUN_TEST(test_pitch_pipeline_with_tilt);
  return UNITY_END();
}
