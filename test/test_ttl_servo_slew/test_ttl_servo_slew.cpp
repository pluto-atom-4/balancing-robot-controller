#include <unity.h>
#include "../../wokwi/chips/ttl_servo_slew.h"

// Sim-only XL330 position-mode slew model. Numbers hand-checked.
// Profile Velocity unit UNVERIFIED against the XL330 e-Manual.

static const double TPS = 0.229 * 4096.0 / 60.0;  // ticks per second per unit (15.6331...)
static const double DT = 0.02;

void setUp(void) {}

void tearDown(void) {}

// Test a: V=100 upward, 2048 -> 2400. Per-tick step 31.266.
void test_slew_up_v100_carries_frac(void) {
  double frac = 0.0;
  uint32_t pos = 2048;
  const uint32_t goal = 2400;
  for (int i = 0; i < 11; i++) {
    pos = ttl_servo_slew_step(pos, goal, 100, TPS, DT, &frac);
  }
  TEST_ASSERT_EQUAL_UINT32(2391, pos);
  TEST_ASSERT_NOT_EQUAL_UINT32(2400, pos);
  pos = ttl_servo_slew_step(pos, goal, 100, TPS, DT, &frac);
  TEST_ASSERT_EQUAL_UINT32(2400, pos);
  pos = ttl_servo_slew_step(pos, goal, 100, TPS, DT, &frac);
  TEST_ASSERT_EQUAL_UINT32(2400, pos);
}

// Test b: V=100 downward, 2400 -> 2048.
void test_slew_down_v100_carries_frac(void) {
  double frac = 0.0;
  uint32_t pos = 2400;
  const uint32_t goal = 2048;
  for (int i = 0; i < 11; i++) {
    pos = ttl_servo_slew_step(pos, goal, 100, TPS, DT, &frac);
  }
  TEST_ASSERT_EQUAL_UINT32(2057, pos);
  pos = ttl_servo_slew_step(pos, goal, 100, TPS, DT, &frac);
  TEST_ASSERT_EQUAL_UINT32(2048, pos);
}

// Test c: V=0 jumps straight to the goal.
void test_slew_v0_jumps_to_goal(void) {
  double frac = 0.0;
  uint32_t pos = ttl_servo_slew_step(2048, 2400, 0, TPS, DT, &frac);
  TEST_ASSERT_EQUAL_UINT32(2400, pos);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, (float)frac);
}

// Test d: V=1 (step 0.3127/tick). Fractional accumulation.
void test_slew_v1_fractional_accumulation(void) {
  double frac = 0.0;
  uint32_t pos = 2048;
  for (int i = 0; i < 3; i++) {
    pos = ttl_servo_slew_step(pos, 3000, 1, TPS, DT, &frac);
  }
  TEST_ASSERT_EQUAL_UINT32(2048, pos);
  pos = ttl_servo_slew_step(pos, 3000, 1, TPS, DT, &frac);
  TEST_ASSERT_EQUAL_UINT32(2049, pos);
  // total 1.2505 ticks, remainder 0.2505
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.25f, (float)frac);
}

// Test e: clamp helper and slew toward an out-of-range goal.
void test_clamp_goal(void) {
  TEST_ASSERT_EQUAL_UINT32(4095, ttl_servo_clamp_goal(5000));
  TEST_ASSERT_EQUAL_UINT32(4095, ttl_servo_clamp_goal(4095));
  TEST_ASSERT_EQUAL_UINT32(0, ttl_servo_clamp_goal(0));
}

void test_slew_goal_clamped_never_exceeds_max(void) {
  double frac = 0.0;
  uint32_t pos = ttl_servo_slew_step(4090, 5000, 0, TPS, DT, &frac);
  TEST_ASSERT_EQUAL_UINT32(4095, pos);
}

// Test a2: finite V toward out-of-range goal, carried frac, never exceeds 4095.
void test_slew_finite_v_out_of_range_goal_ends_at_max(void) {
  double frac = 0.0;
  uint32_t pos = 4090;
  for (int i = 0; i < 50; i++) {
    pos = ttl_servo_slew_step(pos, 5000, 100, TPS, DT, &frac);
    TEST_ASSERT_TRUE(pos <= 4095u);
    if (pos == 4095u) {
      break;
    }
  }
  TEST_ASSERT_EQUAL_UINT32(4095, pos);
}

// Test b2: downward with carried fraction, V=1.
void test_slew_down_v1_carries_negative_frac(void) {
  double frac = 0.0;
  uint32_t pos = 2048;
  for (int i = 0; i < 4; i++) {
    pos = ttl_servo_slew_step(pos, 1000, 1, TPS, DT, &frac);
  }
  TEST_ASSERT_EQUAL_UINT32(2047, pos);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.25f, (float)frac);
}

// Test c2: goal 0 edge, no wrap below 0.
void test_slew_goal_zero_no_wrap(void) {
  double frac = 0.0;
  uint32_t pos = ttl_servo_slew_step(20, 0, 100, TPS, DT, &frac);
  TEST_ASSERT_EQUAL_UINT32(0, pos);
}

// Test d2: huge profile velocity, single call reaches goal.
void test_slew_huge_v_reaches_goal_in_one_call(void) {
  double frac = 0.0;
  uint32_t pos = ttl_servo_slew_step(2048, 2400, 4294967295u, TPS, DT, &frac);
  TEST_ASSERT_EQUAL_UINT32(2400, pos);
}

// Test e2: direction reversal drops the stale positive fraction.
void test_slew_reversal_drops_stale_frac(void) {
  double frac = 0.0;
  uint32_t pos = 2048;
  for (int i = 0; i < 4; i++) {
    pos = ttl_servo_slew_step(pos, 3000, 1, TPS, DT, &frac);
  }
  TEST_ASSERT_EQUAL_UINT32(2049, pos);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.25f, (float)frac);
  pos = ttl_servo_slew_step(pos, 1000, 1, TPS, DT, &frac);
  TEST_ASSERT_EQUAL_UINT32(2049, pos);
  // stale +0.25 dropped; only the new -0.3127 step remains
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.3127f, (float)frac);
}

// Test f: pos == goal resets a stale remainder.
void test_slew_pos_equals_goal_resets_frac(void) {
  double frac = 0.7;
  uint32_t pos = ttl_servo_slew_step(2400, 2400, 100, TPS, DT, &frac);
  TEST_ASSERT_EQUAL_UINT32(2400, pos);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, (float)frac);
}

// Test g: tiny remainder, no overshoot.
void test_slew_tiny_remainder_no_overshoot(void) {
  double frac = 0.0;
  uint32_t pos = ttl_servo_slew_step(2395, 2400, 100, TPS, DT, &frac);
  TEST_ASSERT_EQUAL_UINT32(2400, pos);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_slew_up_v100_carries_frac);
  RUN_TEST(test_slew_down_v100_carries_frac);
  RUN_TEST(test_slew_v0_jumps_to_goal);
  RUN_TEST(test_slew_v1_fractional_accumulation);
  RUN_TEST(test_clamp_goal);
  RUN_TEST(test_slew_goal_clamped_never_exceeds_max);
  RUN_TEST(test_slew_pos_equals_goal_resets_frac);
  RUN_TEST(test_slew_tiny_remainder_no_overshoot);
  RUN_TEST(test_slew_finite_v_out_of_range_goal_ends_at_max);
  RUN_TEST(test_slew_down_v1_carries_negative_frac);
  RUN_TEST(test_slew_goal_zero_no_wrap);
  RUN_TEST(test_slew_huge_v_reaches_goal_in_one_call);
  RUN_TEST(test_slew_reversal_drops_stale_frac);
  return UNITY_END();
}
