#include <unity.h>
#include <math.h>
#include <segway_sim.h>

using namespace segway;

static const float kDt = 0.02f;
static const float kRad2Deg = 57.29578f;

void setUp(void) {}
void tearDown(void) {}

// Gate 1: from 15 deg with the closed loop, settle within 1 deg in 3 s, no fall.
void test_converges_from_15_deg(void) {
  SegwaySim s(15.0f / kRad2Deg);
  for (int i = 0; i < 150; ++i) {
    s.tick(0.0f, kDt);
    TEST_ASSERT_FALSE(s.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(s.theta_rad) * kRad2Deg < 1.0f);
  TEST_ASSERT_TRUE(fabsf(s.theta_dot_rad_s) < 0.05f);
  // At rest the wheel must be (nearly) stopped: servo within 2 deg of 90.
  int servo = s.tick(0.0f, kDt);
  TEST_ASSERT_TRUE(servo >= 88 && servo <= 92);
  TEST_ASSERT_TRUE(fabsf(s.wheel_vel_rad_s) < 0.1f);
}

// Symmetry: -15 deg start and a -1 rad/s impulse behave as the mirror image.
void test_symmetric_negative_cases(void) {
  SegwaySim a(-15.0f / kRad2Deg);
  for (int i = 0; i < 150; ++i) {
    a.tick(0.0f, kDt);
    TEST_ASSERT_FALSE(a.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(a.theta_rad) * kRad2Deg < 1.0f);

  SegwaySim b(0.0f);
  for (int i = 0; i < 150; ++i) {
    b.tick((i >= 10 && i < 15) ? -1.0f : 0.0f, kDt);
    TEST_ASSERT_FALSE(b.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(b.theta_rad) * kRad2Deg < 1.0f);
}

// dt jitter: Wokwi/C3 loops are not exactly 20 ms (Webots alternates 16/32).
void test_converges_with_dt_jitter(void) {
  SegwaySim s(15.0f / kRad2Deg);
  for (int i = 0; i < 200; ++i) {
    s.tick(0.0f, (i % 2) ? 0.032f : 0.016f);
    TEST_ASSERT_FALSE(s.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(s.theta_rad) * kRad2Deg < 1.0f);
}

// The documented stability conditions are real: breaking b*Kp > a falls.
void test_low_gain_violates_stability_condition(void) {
  SegwaySim s(15.0f / kRad2Deg);
  s.params.kp = 2.0f;  // b*Kp = 4 < a = 5
  s.params.kd = 1.0f;
  for (int i = 0; i < 300 && !s.has_fallen; ++i) s.tick(0.0f, kDt);
  TEST_ASSERT_TRUE(s.has_fallen);
}

// Design range: a 30 deg lean is still recoverable with the default gains.
void test_recovers_from_30_deg(void) {
  SegwaySim s(30.0f / kRad2Deg);
  for (int i = 0; i < 250; ++i) {
    s.tick(0.0f, kDt);
    TEST_ASSERT_FALSE(s.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(s.theta_rad) * kRad2Deg < 1.0f);
}

// Gate 2: 1 rad/s gyro Z impulse for 100 ms at rest, recovers within 3 s.
void test_recovers_from_gyro_z_impulse(void) {
  SegwaySim s(0.0f);
  float peak = 0.0f;
  for (int i = 0; i < 150; ++i) {
    float gz = (i >= 10 && i < 15) ? 1.0f : 0.0f;
    s.tick(gz, kDt);
    TEST_ASSERT_FALSE(s.has_fallen);
    if (fabsf(s.theta_rad) > peak) peak = fabsf(s.theta_rad);
  }
  TEST_ASSERT_TRUE(peak * kRad2Deg > 0.2f);  // the impulse really disturbed it
  TEST_ASSERT_TRUE(fabsf(s.theta_rad) * kRad2Deg < 1.0f);
}

// Gate 3: with the wheel stopped the plant falls from 15 deg.
void test_open_loop_falls(void) {
  SegwaySim s(15.0f / kRad2Deg);
  for (int i = 0; i < 150 && !s.has_fallen; ++i) s.step(0.0f, 0.0f, kDt);
  TEST_ASSERT_TRUE(s.has_fallen);
}

// Gate 4: sign. Forward lean -> servo above 90 (forward); back lean -> below.
void test_servo_sign(void) {
  Params p;
  TEST_ASSERT_EQUAL_INT(90, wheel_to_servo_deg(p, pd_wheel_cmd(p, 0.0f, 0.0f)));
  TEST_ASSERT_TRUE(wheel_to_servo_deg(p, pd_wheel_cmd(p, 0.1f, 0.0f)) > 90);
  TEST_ASSERT_TRUE(wheel_to_servo_deg(p, pd_wheel_cmd(p, -0.1f, 0.0f)) < 90);
  TEST_ASSERT_TRUE(wheel_to_servo_deg(p, pd_wheel_cmd(p, 0.0f, 0.2f)) > 90);
}

// Gate 4b: the first tick from a forward lean spins the wheel forward and
// that reduces theta_ddot versus an idle wheel.
void test_forward_wheel_opposes_forward_lean(void) {
  SegwaySim idle(0.2f), driven(0.2f);
  for (int i = 0; i < 5; ++i) {
    idle.step(0.0f, 0.0f, kDt);
    int servo = driven.tick(0.0f, kDt);
    TEST_ASSERT_TRUE(servo > 90);
  }
  TEST_ASSERT_TRUE(driven.theta_dot_rad_s < idle.theta_dot_rad_s);
  TEST_ASSERT_TRUE(driven.wheel_vel_rad_s > 0.0f);
}

// Gate 5: servo/wheel mapping round trip, clamp, dt guards.
void test_mapping_and_guards(void) {
  Params p;
  TEST_ASSERT_EQUAL_INT(180, wheel_to_servo_deg(p, 100.0f));
  TEST_ASSERT_EQUAL_INT(0, wheel_to_servo_deg(p, -100.0f));
  TEST_ASSERT_EQUAL_INT(90, wheel_to_servo_deg(p, NAN));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, servo_deg_to_wheel(p, 90));
  TEST_ASSERT_EQUAL_FLOAT(p.wheel_max_rad_s, servo_deg_to_wheel(p, 180));
  TEST_ASSERT_EQUAL_FLOAT(-p.wheel_max_rad_s, servo_deg_to_wheel(p, 0));
  TEST_ASSERT_EQUAL_FLOAT(p.wheel_max_rad_s, servo_deg_to_wheel(p, 500));
  for (int d = 0; d <= 180; ++d)
    TEST_ASSERT_EQUAL_INT(d, wheel_to_servo_deg(p, servo_deg_to_wheel(p, d)));

  Params bad;
  bad.wheel_max_rad_s = 0.0f;
  TEST_ASSERT_EQUAL_INT(90, wheel_to_servo_deg(bad, 1.0f));
  SegwaySim zt(0.1f);
  zt.params.motor_tau_s = 0.0f;
  zt.step(1.0f, 0.0f, kDt);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, zt.wheel_vel_rad_s);

  SegwaySim s(0.1f);
  s.step(1.0f, 5.0f, 0.0f);
  s.step(1.0f, 5.0f, -1.0f);
  s.step(1.0f, 5.0f, NAN);
  TEST_ASSERT_EQUAL_FLOAT(0.1f, s.theta_rad);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, s.wheel_vel_rad_s);
}

// Gate 6: a sustained gyro Z (Wokwi slider held) gives a bounded offset, no
// fall, and the servo holds off 90 in the direction of the push.
void test_sustained_gyro_z_bounded(void) {
  SegwaySim s(0.0f);
  int servo = 90;
  for (int i = 0; i < 400; ++i) {
    servo = s.tick(1.0f, kDt);
    TEST_ASSERT_FALSE(s.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(s.theta_rad) * kRad2Deg < 25.0f);
  // Positive gyro Z pushes theta forward; the wheel must drive forward (> 90).
  TEST_ASSERT_TRUE(s.theta_rad > 0.0f);
  TEST_ASSERT_TRUE(servo > 90);
}

void test_gyro_deadband_clamp_and_nan(void) {
  Params p;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, disturbance_from_gyro(p, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(p.gyro_clamp_rad_s * p.dist_gain,
                          disturbance_from_gyro(p, 99.0f));
  TEST_ASSERT_EQUAL_FLOAT(-p.gyro_clamp_rad_s * p.dist_gain,
                          disturbance_from_gyro(p, -99.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, disturbance_from_gyro(p, NAN));
}

void test_fall_and_reset(void) {
  SegwaySim s(1.0f);
  s.step(0.0f, 0.0f, kDt);
  TEST_ASSERT_TRUE(s.has_fallen);
  float t = s.theta_rad;
  s.step(0.0f, 0.0f, kDt);  // no-op while fallen
  TEST_ASSERT_EQUAL_FLOAT(t, s.theta_rad);
  s.reset(0.262f);
  TEST_ASSERT_FALSE(s.has_fallen);
  TEST_ASSERT_EQUAL_FLOAT(0.262f, s.theta_rad);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, s.wheel_vel_rad_s);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_converges_from_15_deg);
  RUN_TEST(test_symmetric_negative_cases);
  RUN_TEST(test_converges_with_dt_jitter);
  RUN_TEST(test_low_gain_violates_stability_condition);
  RUN_TEST(test_recovers_from_30_deg);
  RUN_TEST(test_recovers_from_gyro_z_impulse);
  RUN_TEST(test_open_loop_falls);
  RUN_TEST(test_servo_sign);
  RUN_TEST(test_forward_wheel_opposes_forward_lean);
  RUN_TEST(test_mapping_and_guards);
  RUN_TEST(test_sustained_gyro_z_bounded);
  RUN_TEST(test_gyro_deadband_clamp_and_nan);
  RUN_TEST(test_fall_and_reset);
  return UNITY_END();
}
