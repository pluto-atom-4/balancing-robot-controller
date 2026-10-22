#include <unity.h>
#include <segway_imu.h>
#include <tilt.h>
#include <cmath>
#include <limits>

// SIM-ONLY synthetic IMU state (#186). Native tests only; not hardware-validated.

void setUp(void) {}

void tearDown(void) {}

static const float kPiF = 3.14159265358979f;
static const float kDegToRad = kPiF / 180.0f;
static const float kDt = 0.02f;

static void rest(float accel[3], float gyro[3]) {
  segway_imu::from_state(0.0f, 0.0f, accel, gyro);
}

// Test 1: theta = 0 gives the exact rest vector, with no negative zero anywhere
void test_rest_vector_exact(void) {
  float a[3], g[3];
  rest(a, g);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, a[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, a[1]);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, a[2]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[1]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[2]);
  for (int i = 0; i < 3; i++) {
    TEST_ASSERT_FALSE(std::signbit(a[i]));
    TEST_ASSERT_FALSE(std::signbit(g[i]));
  }
}

// Test 2: |a| ~ 1 g for several thetas
void test_accel_magnitude_is_one_g(void) {
  const float degs[] = {-89.0f, -45.0f, -15.0f, -1.0f, 0.0f, 1.0f, 15.0f, 30.0f, 60.0f, 89.0f};
  for (size_t i = 0; i < sizeof(degs) / sizeof(degs[0]); i++) {
    float a[3], g[3];
    segway_imu::from_state(degs[i] * kDegToRad, 0.0f, a, g);
    float mag = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mag);
  }
}

// Test 3: +15 deg table row (ax -0.2588, ay 0, az 0.9659); gy == 0 at theta_dot 0
void test_table_row_plus_15deg(void) {
  float a[3], g[3];
  segway_imu::from_state(15.0f * kDegToRad, 0.0f, a, g);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.2588f, a[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, a[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.9659f, a[2]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[1]);
}

// Test 4: -15 deg table row (ax +0.2588, az 0.9659), gy follows theta_dot sign
void test_table_row_minus_15deg(void) {
  float a[3], g[3];
  segway_imu::from_state(-15.0f * kDegToRad, 0.0f, a, g);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2588f, a[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, a[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.9659f, a[2]);
  segway_imu::from_state(-15.0f * kDegToRad, -0.5f, a, g);
  TEST_ASSERT_TRUE(g[1] < 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.5f * segway_imu::kRadToDeg, g[1]);
}

// Test 5: gz is exactly 0 (and not -0.0f) for representative inputs
void test_gz_always_zero(void) {
  float a[3], g[3];
  segway_imu::from_state(0.0f, 0.0f, a, g);
  TEST_ASSERT_FALSE(std::signbit(g[2]));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[2]);
  segway_imu::from_state(15.0f * kDegToRad, 1.0f, a, g);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[2]);
  segway_imu::from_state(-15.0f * kDegToRad, -10.0f, a, g);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[2]);
  segway_imu::from_state(std::numeric_limits<float>::quiet_NaN(), 0.0f, a, g);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[2]);
}

// Test 6: gyro conversion, 1 rad/s -> 57.29578 deg/s
void test_gy_conversion(void) {
  float a[3], g[3];
  segway_imu::from_state(0.0f, 1.0f, a, g);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 57.29578f, g[1]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[0]);
}

// Test 7: gyro clamp at +-250 deg/s
void test_gyro_clamp(void) {
  float a[3], g[3];
  segway_imu::from_state(0.0f, 10.0f, a, g);
  TEST_ASSERT_EQUAL_FLOAT(250.0f, g[1]);
  segway_imu::from_state(0.0f, -10.0f, a, g);
  TEST_ASSERT_EQUAL_FLOAT(-250.0f, g[1]);
}

// Test 9: NaN theta is treated as 0 (rest vector); NaN theta_dot as 0
void test_nan_inputs_become_zero(void) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  float a[3], g[3];
  segway_imu::from_state(nan, 0.0f, a, g);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, a[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, a[1]);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, a[2]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[1]);

  segway_imu::from_state(0.0f, nan, a, g);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[1]);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, a[2]);
}

// Test 10: +-inf theta and theta_dot are treated as 0
void test_inf_inputs_become_zero(void) {
  const float inf = std::numeric_limits<float>::infinity();
  float a[3], g[3];
  segway_imu::from_state(inf, 0.0f, a, g);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, a[0]);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, a[2]);

  segway_imu::from_state(-inf, inf, a, g);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, a[0]);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, a[2]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[1]);

  segway_imu::from_state(0.0f, -inf, a, g);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g[1]);
}

// Test 11: S0 section-5 round trip. Ramp 0 -> +20 deg at 20 deg/s (1 s, 0.02 s
// steps) through tilt::accelPitchDeg and tilt::ComplementaryFilter, then back.
// Fused pitch must equal theta within 0.5 deg, and sign(gy) == sign(theta_dot).
static void rampStep(tilt::ComplementaryFilter& f, float thetaDeg, float thetaDotDegPerS,
                     float& gyroOut) {
  float a[3], g[3];
  segway_imu::from_state(thetaDeg * kDegToRad, thetaDotDegPerS * kDegToRad, a, g);
  gyroOut = g[1];
  f.update(tilt::accelPitchDeg(a[0], a[1], a[2]), g[1], kDt);
}

void test_round_trip_forward_and_reverse_ramp(void) {
  tilt::ComplementaryFilter f;
  float gy = 0.0f;
  const int steps = 50;  // 1 s at 0.02 s per step
  const float rate = 20.0f;  // deg/s

  // Forward: 0 -> +20 deg
  rampStep(f, 0.0f, rate, gy);
  bool forwardSignOk = true;
  for (int i = 1; i <= steps; i++) {
    const float theta = rate * static_cast<float>(i) * kDt;
    rampStep(f, theta, rate, gy);
    if (!(gy > 0.0f)) forwardSignOk = false;
  }
  TEST_ASSERT_TRUE(forwardSignOk);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, f.angle);

  // Reverse: +20 -> 0 deg
  bool reverseSignOk = true;
  for (int i = 1; i <= steps; i++) {
    const float theta = 20.0f - rate * static_cast<float>(i) * kDt;
    rampStep(f, theta, -rate, gy);
    if (!(gy < 0.0f)) reverseSignOk = false;
  }
  TEST_ASSERT_TRUE(reverseSignOk);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, f.angle);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_rest_vector_exact);
  RUN_TEST(test_accel_magnitude_is_one_g);
  RUN_TEST(test_table_row_plus_15deg);
  RUN_TEST(test_table_row_minus_15deg);
  RUN_TEST(test_gz_always_zero);
  RUN_TEST(test_gy_conversion);
  RUN_TEST(test_gyro_clamp);
  RUN_TEST(test_nan_inputs_become_zero);
  RUN_TEST(test_inf_inputs_become_zero);
  RUN_TEST(test_round_trip_forward_and_reverse_ramp);
  return UNITY_END();
}
