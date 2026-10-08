#include <unity.h>
#include <segway_sim.h>
#include <cmath>

void test_constructor(void) {
  SegwaySim sim(0.262f);
  TEST_ASSERT_EQUAL_FLOAT(0.262f, sim.theta_rad);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.theta_dot_rad_s);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.wheel_vel_rad_s);
  TEST_ASSERT_FALSE(sim.has_fallen);
}

void test_reset(void) {
  SegwaySim sim(0.262f);
  for (int i = 0; i < 10; ++i) {
    sim.step(1.0f, 5.0f, 0.02f);
  }
  sim.reset(0.262f);
  TEST_ASSERT_EQUAL_FLOAT(0.262f, sim.theta_rad);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.theta_dot_rad_s);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.wheel_vel_rad_s);
  TEST_ASSERT_FALSE(sim.has_fallen);
}

void test_zero_dt(void) {
  SegwaySim sim(0.1f);
  float theta_orig = sim.theta_rad;
  sim.step(1.0f, 5.0f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(theta_orig, sim.theta_rad);
}

void test_motor_lag(void) {
  SegwaySim sim(0.0f);
  float wheel_target = 1.0f;
  sim.step(wheel_target, 0.0f, 0.02f);
  float wheel_vel_1 = sim.wheel_vel_rad_s;
  // Motor lag: wheel_vel should be positive but less than target
  TEST_ASSERT_TRUE(wheel_vel_1 > 0.0f && wheel_vel_1 < wheel_target);
}

void test_fall_detection(void) {
  SegwaySim sim(0.0f);
  sim.theta_rad = 1.0f;  // Exceed fall threshold (0.785 rad)
  sim.step(0.0f, 0.0f, 0.02f);
  TEST_ASSERT_TRUE(sim.has_fallen);
}

void test_pendulum_unstable(void) {
  SegwaySim sim(0.01f);
  float theta_dot_initial = sim.theta_dot_rad_s;
  sim.step(0.0f, 0.0f, 0.02f);
  // Gravity should push it: theta_dot should increase
  TEST_ASSERT_TRUE(sim.theta_dot_rad_s > theta_dot_initial);
}

void test_gravity_pos_theta(void) {
  SegwaySim sim(0.1f);
  float theta_dot_initial = sim.theta_dot_rad_s;
  sim.step(0.0f, 0.0f, 0.02f);
  TEST_ASSERT_TRUE(sim.theta_dot_rad_s > theta_dot_initial);
}

void test_gravity_neg_theta(void) {
  SegwaySim sim(-0.1f);
  float theta_dot_initial = sim.theta_dot_rad_s;
  sim.step(0.0f, 0.0f, 0.02f);
  TEST_ASSERT_TRUE(sim.theta_dot_rad_s < theta_dot_initial);
}

void test_disturbance(void) {
  SegwaySim sim1(0.0f);
  SegwaySim sim2(0.0f);
  for (int i = 0; i < 10; ++i) {
    sim1.step(0.0f, 0.0f, 0.02f);
    sim2.step(0.0f, 5.0f, 0.02f);
  }
  // Disturbance should cause larger tilt
  TEST_ASSERT_TRUE(fabsf(sim2.theta_rad) > fabsf(sim1.theta_rad));
}

void test_free_fall(void) {
  SegwaySim sim(0.262f);  // 15 deg
  for (int i = 0; i < 50 && !sim.has_fallen; ++i) {
    sim.step(0.0f, 0.0f, 0.02f);
  }
  TEST_ASSERT_TRUE(sim.has_fallen);
}

void test_motor_convergence(void) {
  SegwaySim sim(0.0f);
  float wheel_target = 1.0f;
  for (int i = 0; i < 50; ++i) {
    sim.step(wheel_target, 0.0f, 0.02f);
  }
  // Should be very close to target after 1 second
  TEST_ASSERT_TRUE(sim.wheel_vel_rad_s > 0.95f);
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_constructor);
  RUN_TEST(test_reset);
  RUN_TEST(test_zero_dt);
  RUN_TEST(test_motor_lag);
  RUN_TEST(test_fall_detection);
  RUN_TEST(test_pendulum_unstable);
  RUN_TEST(test_gravity_pos_theta);
  RUN_TEST(test_gravity_neg_theta);
  RUN_TEST(test_disturbance);
  RUN_TEST(test_free_fall);
  RUN_TEST(test_motor_convergence);
  return UNITY_END();
}
