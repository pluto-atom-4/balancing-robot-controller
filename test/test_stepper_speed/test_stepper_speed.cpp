#include <unity.h>
#include <math.h>
#include <segway_sim.h>
#include <stepper_speed.h>

using namespace stepper;

static const float kSpr = 3200.0f;  // 200 steps x 1/16 microstep

void setUp(void) {}
void tearDown(void) {}

void test_rate_mapping(void) {
  // 1 rev/s = 2*pi rad/s = 3200 steps/s; 3 rad/s (wheel max) ~ 1528 steps/s.
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 3200.0f, rad_s_to_steps_s(kTwoPi, kSpr));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 1527.9f, rad_s_to_steps_s(3.0f, kSpr));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -1527.9f, rad_s_to_steps_s(-3.0f, kSpr));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, rad_s_to_steps_s(0.0f, kSpr));
}

void test_rate_guards(void) {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, rad_s_to_steps_s(NAN, kSpr));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, rad_s_to_steps_s(1.0f, 0.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, rad_s_to_steps_s(1.0f, -5.0f));
}

void test_steps_match_rate(void) {
  StepAccumulator a;
  unsigned total = 0;
  for (int i = 0; i < 1000; ++i) total += a.advance(1000.0f, 0.001f, 8);  // 1 s
  TEST_ASSERT_UINT_WITHIN(1, 1000, total);
  TEST_ASSERT_EQUAL_UINT(0, a.dropped());
  TEST_ASSERT_TRUE(a.forward());
  TEST_ASSERT_EQUAL_INT((long)total, a.position());
}

void test_fractional_carry(void) {
  StepAccumulator a;  // 100 steps/s at 1 ms: one step every 10th pass
  unsigned total = 0;
  for (int i = 0; i < 100; ++i) total += a.advance(100.0f, 0.001f, 8);
  TEST_ASSERT_UINT_WITHIN(1, 10, total);
}

void test_direction_and_position(void) {
  StepAccumulator a;
  unsigned f = 0, r = 0;
  for (int i = 0; i < 500; ++i) f += a.advance(1000.0f, 0.001f, 8);
  TEST_ASSERT_TRUE(a.forward());
  for (int i = 0; i < 300; ++i) r += a.advance(-1000.0f, 0.001f, 8);
  TEST_ASSERT_FALSE(a.forward());
  TEST_ASSERT_EQUAL_INT((long)f - (long)r, a.position());
  TEST_ASSERT_UINT_WITHIN(1, 300, r);
}

void test_direction_change_drops_remainder(void) {
  StepAccumulator a;
  TEST_ASSERT_EQUAL_UINT(0, a.advance(100.0f, 0.009f, 8));  // frac 0.9
  TEST_ASSERT_EQUAL_UINT(0, a.advance(-100.0f, 0.002f, 8)); // would be 1.1 without the drop
  TEST_ASSERT_EQUAL_INT(0, a.position());
}

void test_zero_holds(void) {
  StepAccumulator a;
  for (int i = 0; i < 100; ++i) TEST_ASSERT_EQUAL_UINT(0, a.advance(0.0f, 0.001f, 8));
  TEST_ASSERT_EQUAL_INT(0, a.position());
  TEST_ASSERT_EQUAL_UINT(0, a.dropped());
}

void test_cap_counts_dropped_no_catchup(void) {
  StepAccumulator a;
  unsigned n = a.advance(1000.0f, 0.1f, 8);  // 100 steps demanded, cap 8
  TEST_ASSERT_EQUAL_UINT(8, n);
  TEST_ASSERT_EQUAL_UINT(92, a.dropped());
  // No catch-up: the next small pass emits only its own share.
  TEST_ASSERT_EQUAL_UINT(1, a.advance(1000.0f, 0.001f, 8));
  TEST_ASSERT_EQUAL_UINT(92, a.dropped());
}

void test_guards(void) {
  StepAccumulator a;
  TEST_ASSERT_EQUAL_UINT(0, a.advance(NAN, 0.001f, 8));
  TEST_ASSERT_EQUAL_UINT(0, a.advance(1000.0f, 0.0f, 8));
  TEST_ASSERT_EQUAL_UINT(0, a.advance(1000.0f, -1.0f, 8));
  TEST_ASSERT_EQUAL_UINT(0, a.advance(1000.0f, NAN, 8));
  unsigned n = a.advance(INFINITY, 0.001f, 8);  // must not hang or overflow
  TEST_ASSERT_TRUE(n <= 8);
  TEST_ASSERT_EQUAL_UINT(0, a.advance(1000.0f, 0.001f, 0));  // cap 0 emits nothing
}

void test_reset_and_hold(void) {
  StepAccumulator a;
  a.advance(1000.0f, 0.05f, 100);
  a.hold();
  TEST_ASSERT_TRUE(a.position() > 0);
  a.reset();
  TEST_ASSERT_EQUAL_INT(0, a.position());
  TEST_ASSERT_EQUAL_UINT(0, a.emitted());
  TEST_ASSERT_EQUAL_UINT(0, a.dropped());
}

// Closed loop: 50 Hz control tick drives the sim, 1 kHz step service emits steps.
// Gate: steps follow the integral of wheel velocity, speed varies and reverses,
// the motor is nearly still after settling, and the cap is never hit.
void test_closed_loop_follows_wheel(void) {
  segway::SegwaySim s(15.0f / 57.29578f);
  StepAccumulator acc;
  float integral_steps = 0.0f, smax = -1e9f, smin = 1e9f;
  long pos_at_3s = 0;
  float sps = 0.0f;
  for (int ms = 0; ms < 6000; ++ms) {
    if (ms % 20 == 0) {
      s.step(segway::pd_wheel_cmd(s.params, s.theta_rad, s.theta_dot_rad_s), 0.0f, 0.02f);
      TEST_ASSERT_FALSE(s.has_fallen);
      sps = rad_s_to_steps_s(s.wheel_vel_rad_s, kSpr);
      if (sps > smax) smax = sps;
      if (sps < smin) smin = sps;
    }
    integral_steps += sps * 0.001f;
    acc.advance(sps, 0.001f, 8);
    if (ms == 3000) pos_at_3s = acc.position();
  }
  TEST_ASSERT_EQUAL_UINT(0, acc.dropped());
  TEST_ASSERT_FLOAT_WITHIN(2.0f, integral_steps, (float)acc.position());
  // Observed: peak +796 steps/s (1.56 rad/s), reverses to -144 steps/s.
  TEST_ASSERT_TRUE(smax > 600.0f);   // spun up forward
  TEST_ASSERT_TRUE(smin < -75.0f);   // then reversed
  TEST_ASSERT_TRUE(labs(acc.position() - pos_at_3s) < 100);  // still after 3 s
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_rate_mapping);
  RUN_TEST(test_rate_guards);
  RUN_TEST(test_steps_match_rate);
  RUN_TEST(test_fractional_carry);
  RUN_TEST(test_direction_and_position);
  RUN_TEST(test_direction_change_drops_remainder);
  RUN_TEST(test_zero_holds);
  RUN_TEST(test_cap_counts_dropped_no_catchup);
  RUN_TEST(test_guards);
  RUN_TEST(test_reset_and_hold);
  RUN_TEST(test_closed_loop_follows_wheel);
  return UNITY_END();
}
