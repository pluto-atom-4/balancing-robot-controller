#include <unity.h>
#include <math.h>
#include <segway_sim.h>
#include <dxl_units.h>

using namespace segway;

static const float kDt = 0.02f;
static const float kRad2Deg = 57.29578f;

void setUp(void) {}
void tearDown(void) {}

// Gate 1: from 15 deg with the closed loop, settle within 1 deg in 3 s, no fall.
void test_converges_from_15_deg(void) {
  SegwaySim s(15.0f / kRad2Deg);
  for (int i = 0; i < 150; ++i) {
    s.tick_cmd(0.0f, kDt);
    TEST_ASSERT_FALSE(s.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(s.theta_rad) * kRad2Deg < 1.0f);
  TEST_ASSERT_TRUE(fabsf(s.theta_dot_rad_s) < 0.05f);
  // At rest the wheel command must be (nearly) zero.
  float cmd = s.tick_cmd(0.0f, kDt);
  TEST_ASSERT_TRUE(fabsf(cmd) < 0.1f);
  TEST_ASSERT_TRUE(fabsf(s.wheel_vel_rad_s) < 0.1f);
}

// Symmetry: -15 deg start and a -1 rad/s impulse behave as the mirror image.
void test_symmetric_negative_cases(void) {
  SegwaySim a(-15.0f / kRad2Deg);
  for (int i = 0; i < 150; ++i) {
    a.tick_cmd(0.0f, kDt);
    TEST_ASSERT_FALSE(a.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(a.theta_rad) * kRad2Deg < 1.0f);

  SegwaySim b(0.0f);
  for (int i = 0; i < 150; ++i) {
    b.tick_cmd((i >= 10 && i < 15) ? -1.0f : 0.0f, kDt);
    TEST_ASSERT_FALSE(b.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(b.theta_rad) * kRad2Deg < 1.0f);
}

// dt jitter: Wokwi/C3 loops are not exactly 20 ms (Webots alternates 16/32).
void test_converges_with_dt_jitter(void) {
  SegwaySim s(15.0f / kRad2Deg);
  for (int i = 0; i < 200; ++i) {
    s.tick_cmd(0.0f, (i % 2) ? 0.032f : 0.016f);
    TEST_ASSERT_FALSE(s.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(s.theta_rad) * kRad2Deg < 1.0f);
}

// The documented stability conditions are real: breaking b*Kp > a falls.
void test_low_gain_violates_stability_condition(void) {
  SegwaySim s(15.0f / kRad2Deg);
  s.params.kp = 2.0f;  // b*Kp = 4 < a = 5
  s.params.kd = 1.0f;
  for (int i = 0; i < 300 && !s.has_fallen; ++i) s.tick_cmd(0.0f, kDt);
  TEST_ASSERT_TRUE(s.has_fallen);
}

// Design range: a 30 deg lean is still recoverable with the default gains.
void test_recovers_from_30_deg(void) {
  SegwaySim s(30.0f / kRad2Deg);
  for (int i = 0; i < 250; ++i) {
    s.tick_cmd(0.0f, kDt);
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
    s.tick_cmd(gz, kDt);
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

// Gate 4: wheel command sign. Forward lean -> positive command (forward); back lean -> negative.
void test_wheel_cmd_sign(void) {
  Params p;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pd_wheel_cmd(p, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(pd_wheel_cmd(p, 0.1f, 0.0f) > 0.0f);
  TEST_ASSERT_TRUE(pd_wheel_cmd(p, -0.1f, 0.0f) < 0.0f);
  TEST_ASSERT_TRUE(pd_wheel_cmd(p, 0.0f, 0.2f) > 0.0f);
  TEST_ASSERT_TRUE(wheel_cmd_to_raw(pd_wheel_cmd(p, 0.1f, 0.0f)) > 0);
  TEST_ASSERT_TRUE(wheel_cmd_to_raw(pd_wheel_cmd(p, -0.1f, 0.0f)) < 0);
}

// Gate 4b: the first tick from a forward lean spins the wheel forward and
// that reduces theta_ddot versus an idle wheel.
void test_forward_wheel_opposes_forward_lean(void) {
  SegwaySim idle(0.2f), driven(0.2f);
  for (int i = 0; i < 5; ++i) {
    idle.step(0.0f, 0.0f, kDt);
    float cmd = driven.tick_cmd(0.0f, kDt);
    TEST_ASSERT_TRUE(cmd > 0.0f);
  }
  TEST_ASSERT_TRUE(driven.theta_dot_rad_s < idle.theta_dot_rad_s);
  TEST_ASSERT_TRUE(driven.wheel_vel_rad_s > 0.0f);
}

// Gate 5: zero motor tau, dt guards.
void test_mapping_and_guards(void) {
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
// fall, and the wheel command holds off zero in the direction of the push.
void test_sustained_gyro_z_bounded(void) {
  SegwaySim s(0.0f);
  float cmd = 0.0f;
  for (int i = 0; i < 400; ++i) {
    cmd = s.tick_cmd(1.0f, kDt);
    TEST_ASSERT_FALSE(s.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(s.theta_rad) * kRad2Deg < 25.0f);
  // Positive gyro Z pushes theta forward; the wheel must drive forward (> 0).
  TEST_ASSERT_TRUE(s.theta_rad > 0.0f);
  TEST_ASSERT_TRUE(cmd > 0.0f);
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

// tick_cmd: the PD command (rad/s) from 15 deg converges within 3 s, no fall.
void test_tick_cmd_converges_from_15_deg(void) {
  SegwaySim s(0.2618f);
  float first = s.tick_cmd(0.0f, 0.02f);
  TEST_ASSERT_TRUE(first > 0.0f);
  for (int i = 0; i < 149; ++i) {
    s.tick_cmd(0.0f, 0.02f);
    TEST_ASSERT_FALSE(s.has_fallen);
  }
  TEST_ASSERT_TRUE(fabsf(s.theta_rad) < 0.01745f);
  TEST_ASSERT_TRUE(fabsf(s.wheel_vel_rad_s) < 0.1f);
}

// wheel_cmd_to_raw: rad/s -> XL330 raw velocity, rounded, clamped to the cap.
void test_wheel_cmd_to_raw_mapping(void) {
  TEST_ASSERT_EQUAL_INT(0, wheel_cmd_to_raw(0.0f));
  TEST_ASSERT_EQUAL_INT(42, wheel_cmd_to_raw(1.0f));
  TEST_ASSERT_EQUAL_INT(125, wheel_cmd_to_raw(3.0f));
  TEST_ASSERT_EQUAL_INT(-125, wheel_cmd_to_raw(-3.0f));
  TEST_ASSERT_EQUAL_INT(125, wheel_cmd_to_raw(100.0f));
  TEST_ASSERT_EQUAL_INT(0, wheel_cmd_to_raw(__builtin_nanf("")));
  TEST_ASSERT_EQUAL_INT(4, wheel_cmd_to_raw(0.1f));
  TEST_ASSERT_EQUAL_INT(0, wheel_cmd_to_raw(0.0119f));
  TEST_ASSERT_EQUAL_INT(1, wheel_cmd_to_raw(0.0240f));
  int raw90 = wheel_cmd_to_raw(1.5708f);  // 90 deg/s
  TEST_ASSERT_TRUE(raw90 >= 65 && raw90 <= 66);
  const float xs[] = {0.5f, 1.0f, 2.0f};
  for (int k = 0; k < 3; ++k)
    TEST_ASSERT_EQUAL_INT(-wheel_cmd_to_raw(xs[k]), wheel_cmd_to_raw(-xs[k]));
  // Regression: frozen-wheel case when a device-read Velocity Limit is 0.
  TEST_ASSERT_EQUAL_INT(0, wheel_cmd_to_raw(1.0f, 0));
}

// The raw cap matches the wheel speed cap in Params and the XL330 scale.
void test_wheel_cap_matches_params(void) {
  Params p;
  TEST_ASSERT_EQUAL_INT(kWheelCapRaw, wheel_cmd_to_raw(p.wheel_max_rad_s));
  float cap_rad_s = dxl_units::rawToRadS(dxl_units::kXl330, kWheelCapRaw);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, cap_rad_s);
  TEST_ASSERT_TRUE(cap_rad_s <= p.wheel_max_rad_s);
}

// Sign and range of the raw command over a full closed-loop run from +15 deg:
// first forward, the cap is never exceeded, and the sign reverses (overshoot).
void test_tick_cmd_sign_and_range(void) {
  SegwaySim pos(0.2618f);
  TEST_ASSERT_TRUE(wheel_cmd_to_raw(pos.tick_cmd(0.0f, 0.02f)) > 0);

  SegwaySim neg(-0.2618f);
  TEST_ASSERT_TRUE(wheel_cmd_to_raw(neg.tick_cmd(0.0f, 0.02f)) < 0);

  SegwaySim s(0.2618f);
  bool seen_pos = false, seen_neg_after_pos = false;
  for (int i = 0; i < 200; ++i) {
    int raw = wheel_cmd_to_raw(s.tick_cmd(0.0f, 0.02f));
    TEST_ASSERT_TRUE(raw <= 125 && raw >= -125);
    if (raw > 0) seen_pos = true;
    if (seen_pos && raw < 0) seen_neg_after_pos = true;
  }
  TEST_ASSERT_TRUE(seen_pos);
  TEST_ASSERT_TRUE(seen_neg_after_pos);
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
  RUN_TEST(test_wheel_cmd_sign);
  RUN_TEST(test_forward_wheel_opposes_forward_lean);
  RUN_TEST(test_mapping_and_guards);
  RUN_TEST(test_sustained_gyro_z_bounded);
  RUN_TEST(test_gyro_deadband_clamp_and_nan);
  RUN_TEST(test_fall_and_reset);
  RUN_TEST(test_tick_cmd_converges_from_15_deg);
  RUN_TEST(test_wheel_cmd_to_raw_mapping);
  RUN_TEST(test_wheel_cap_matches_params);
  RUN_TEST(test_tick_cmd_sign_and_range);
  return UNITY_END();
}
