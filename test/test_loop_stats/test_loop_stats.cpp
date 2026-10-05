#include <unity.h>
#include <loop_stats.h>
#include <cstdint>

void setUp(void) {}
void tearDown(void) {}

// Empty tracker: mean 0, defaults untouched.
void test_empty(void) {
  loop_stats::LoopStats s;
  TEST_ASSERT_EQUAL_UINT32(0, s.count);
  TEST_ASSERT_EQUAL_UINT32(0, s.mean_us());
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, s.min_us);
  TEST_ASSERT_EQUAL_UINT32(0, s.max_us);
  TEST_ASSERT_EQUAL_UINT32(0, s.over_budget);
}

// Webots-like pattern 16000, 32000, 16000 us, budget 20000 us.
// sum = 64000, count = 3, mean = 64000/3 = 21333.33 -> 21333 (truncated)
// min = 16000, max = 32000, over_budget: only 32000 > 20000 -> 1
void test_known_sequence(void) {
  loop_stats::LoopStats s;
  s.add(16000, 20000);
  s.add(32000, 20000);
  s.add(16000, 20000);
  TEST_ASSERT_EQUAL_UINT32(3, s.count);
  TEST_ASSERT_EQUAL_UINT32(16000, s.min_us);
  TEST_ASSERT_EQUAL_UINT32(32000, s.max_us);
  TEST_ASSERT_EQUAL_UINT32(21333, s.mean_us());
  TEST_ASSERT_EQUAL_UINT32(1, s.over_budget);
  TEST_ASSERT_TRUE(s.sum_us == 64000ULL);
}

// Budget boundary is strict: period == budget is NOT over.
// add(20000,20000) -> over 0; add(20001,20000) -> over 1
// sum = 40001, count 2, mean = 20000.5 -> 20000; min 20000, max 20001
void test_budget_boundary(void) {
  loop_stats::LoopStats s;
  s.add(20000, 20000);
  TEST_ASSERT_EQUAL_UINT32(0, s.over_budget);
  s.add(20001, 20000);
  TEST_ASSERT_EQUAL_UINT32(1, s.over_budget);
  TEST_ASSERT_EQUAL_UINT32(20000, s.min_us);
  TEST_ASSERT_EQUAL_UINT32(20001, s.max_us);
  TEST_ASSERT_EQUAL_UINT32(20000, s.mean_us());
}

// reset() returns to defaults and the tracker works again afterwards.
void test_reset(void) {
  loop_stats::LoopStats s;
  s.add(16000, 10000);
  s.add(32000, 10000);
  s.reset();
  TEST_ASSERT_EQUAL_UINT32(0, s.count);
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, s.min_us);
  TEST_ASSERT_EQUAL_UINT32(0, s.max_us);
  TEST_ASSERT_EQUAL_UINT32(0, s.over_budget);
  TEST_ASSERT_TRUE(s.sum_us == 0ULL);
  TEST_ASSERT_EQUAL_UINT32(0, s.mean_us());
  s.add(7, 10);
  TEST_ASSERT_EQUAL_UINT32(1, s.count);
  TEST_ASSERT_EQUAL_UINT32(7, s.min_us);
  TEST_ASSERT_EQUAL_UINT32(7, s.max_us);
  TEST_ASSERT_EQUAL_UINT32(7, s.mean_us());
  TEST_ASSERT_EQUAL_UINT32(0, s.over_budget);
}

// 1e6 samples of 20000 us: sum = 2e10 > 2^32 (4294967296), so a uint32 sum
// would overflow. mean must still be exactly 20000.
void test_no_sum_overflow(void) {
  loop_stats::LoopStats s;
  for (uint32_t i = 0; i < 1000000u; ++i) s.add(20000, 25000);
  TEST_ASSERT_EQUAL_UINT32(1000000u, s.count);
  TEST_ASSERT_TRUE(s.sum_us == 20000000000ULL);
  TEST_ASSERT_EQUAL_UINT32(20000, s.mean_us());
  TEST_ASSERT_EQUAL_UINT32(0, s.over_budget);
}

// count saturates: at 0xFFFFFFFF a further add() is ignored.
void test_count_saturates(void) {
  loop_stats::LoopStats s;
  s.count = 0xFFFFFFFFu;
  s.sum_us = 1000ULL;
  s.add(5, 1);
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, s.count);
  TEST_ASSERT_TRUE(s.sum_us == 1000ULL);
  TEST_ASSERT_EQUAL_UINT32(0, s.over_budget);
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, s.min_us);
}

// Zero period is a legal sample: min becomes 0, mean = 0.
void test_zero_period(void) {
  loop_stats::LoopStats s;
  s.add(0, 10);
  TEST_ASSERT_EQUAL_UINT32(1, s.count);
  TEST_ASSERT_EQUAL_UINT32(0, s.min_us);
  TEST_ASSERT_EQUAL_UINT32(0, s.max_us);
  TEST_ASSERT_EQUAL_UINT32(0, s.mean_us());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_empty);
  RUN_TEST(test_known_sequence);
  RUN_TEST(test_budget_boundary);
  RUN_TEST(test_reset);
  RUN_TEST(test_no_sum_overflow);
  RUN_TEST(test_count_saturates);
  RUN_TEST(test_zero_period);
  return UNITY_END();
}
