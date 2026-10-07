#include <unity.h>
#include <ramp_guard.h>

void setUp(void) {}

void tearDown(void) {}

// capFromLimit tests
void test_capFromLimit_basic_1620_20(void) {
  // (1620 * 20) / 100 = 324
  TEST_ASSERT_EQUAL_INT32(324, ramp_guard::capFromLimit(1620, 20));
}

void test_capFromLimit_basic_2047_20(void) {
  // (2047 * 20) / 100 = 409
  TEST_ASSERT_EQUAL_INT32(409, ramp_guard::capFromLimit(2047, 20));
}

void test_capFromLimit_basic_1620_25(void) {
  // (1620 * 25) / 100 = 405
  TEST_ASSERT_EQUAL_INT32(405, ramp_guard::capFromLimit(1620, 25));
}

void test_capFromLimit_basic_100_20(void) {
  // (100 * 20) / 100 = 20
  TEST_ASSERT_EQUAL_INT32(20, ramp_guard::capFromLimit(100, 20));
}

void test_capFromLimit_reject_4_20(void) {
  // 4 is in range but result is 0 (cap less than 1)
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::capFromLimit(4, 20));
}

void test_capFromLimit_reject_1_20(void) {
  // 1 <= limit_raw; (1 * 20) / 100 = 0
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::capFromLimit(1, 20));
}

void test_capFromLimit_reject_0(void) {
  // limit_raw <= 0
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::capFromLimit(0, 20));
}

void test_capFromLimit_reject_negative(void) {
  // limit_raw < 0
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::capFromLimit(-5, 20));
}

void test_capFromLimit_reject_2048(void) {
  // limit_raw > kMaxSaneLimitRaw (2047)
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::capFromLimit(2048, 20));
}

void test_capFromLimit_reject_pct_26(void) {
  // pct > kMaxCapPct (25)
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::capFromLimit(1620, 26));
}

void test_capFromLimit_reject_pct_0(void) {
  // pct <= 0
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::capFromLimit(1620, 0));
}

// goalAt tests with cap=324
void test_goalAt_t0_cap324(void) {
  // t=0: (324 * 0) / 7000 = 0
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::goalAt(0, 324));
}

void test_goalAt_t250_cap324(void) {
  // t=250: (324 * 250) / 7000 = 11 (rounding down)
  TEST_ASSERT_EQUAL_INT32(11, ramp_guard::goalAt(250, 324));
}

void test_goalAt_t500_cap324(void) {
  // t=500: (324 * 500) / 7000 = 23
  TEST_ASSERT_EQUAL_INT32(23, ramp_guard::goalAt(500, 324));
}

void test_goalAt_t1000_cap324(void) {
  // t=1000: (324 * 1000) / 7000 = 46
  TEST_ASSERT_EQUAL_INT32(46, ramp_guard::goalAt(1000, 324));
}

void test_goalAt_t3500_cap324(void) {
  // t=3500: (324 * 3500) / 7000 = 162
  TEST_ASSERT_EQUAL_INT32(162, ramp_guard::goalAt(3500, 324));
}

void test_goalAt_t6999_cap324(void) {
  // t=6999: (324 * 6999) / 7000 = 323
  TEST_ASSERT_EQUAL_INT32(323, ramp_guard::goalAt(6999, 324));
}

void test_goalAt_t7000_cap324(void) {
  // t=7000 is at kUpMs, so in hold region: cap = 324
  TEST_ASSERT_EQUAL_INT32(324, ramp_guard::goalAt(7000, 324));
}

void test_goalAt_t7999_cap324(void) {
  // t=7999 is still in hold region: cap = 324
  TEST_ASSERT_EQUAL_INT32(324, ramp_guard::goalAt(7999, 324));
}

void test_goalAt_t8000_cap324(void) {
  // t=8000 is at kUpMs + kHoldMs, now in down region: (324 * (15000 - 8000)) / 7000 = (324 * 7000) / 7000 = 324
  TEST_ASSERT_EQUAL_INT32(324, ramp_guard::goalAt(8000, 324));
}

void test_goalAt_t11500_cap324(void) {
  // t=11500 is in down region: (324 * (15000 - 11500)) / 7000 = (324 * 3500) / 7000 = 162
  TEST_ASSERT_EQUAL_INT32(162, ramp_guard::goalAt(11500, 324));
}

void test_goalAt_t14000_cap324(void) {
  // t=14000 is in down region: (324 * (15000 - 14000)) / 7000 = (324 * 1000) / 7000 = 46
  TEST_ASSERT_EQUAL_INT32(46, ramp_guard::goalAt(14000, 324));
}

void test_goalAt_t14999_cap324(void) {
  // t=14999 is in down region: (324 * (15000 - 14999)) / 7000 = (324 * 1) / 7000 = 0
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::goalAt(14999, 324));
}

void test_goalAt_t15000_cap324(void) {
  // t=15000 >= kTotalMs, return 0
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::goalAt(15000, 324));
}

void test_goalAt_t99999_cap324(void) {
  // t=99999 >= kTotalMs, return 0
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::goalAt(99999, 324));
}

void test_goalAt_t3500_cap0(void) {
  // cap=0, should return 0 immediately
  TEST_ASSERT_EQUAL_INT32(0, ramp_guard::goalAt(3500, 0));
}

void test_goalAt_monotonic_up(void) {
  // Loop t=0..6999 step 1: should be non-decreasing and <= 324
  int32_t prev = 0;
  for (uint32_t t = 0; t < 7000; t++) {
    int32_t val = ramp_guard::goalAt(t, 324);
    TEST_ASSERT_GREATER_OR_EQUAL_INT32(prev, val);
    TEST_ASSERT_LESS_OR_EQUAL_INT32(324, val);  // Unity: (threshold, actual) passes if actual <= threshold
    prev = val;
  }
}

void test_goalAt_monotonic_down(void) {
  // Loop t=8000..14999 step 1: should be non-increasing and >= 0
  int32_t prev = 324;
  for (uint32_t t = 8000; t < 15000; t++) {
    int32_t val = ramp_guard::goalAt(t, 324);
    TEST_ASSERT_LESS_OR_EQUAL_INT32(prev, val);  // val <= prev (non-increasing)
    TEST_ASSERT_GREATER_OR_EQUAL_INT32(0, val);  // val >= 0
    prev = val;
  }
}

// rampDone tests
void test_rampDone_14999(void) {
  // t=14999 < kTotalMs (15000)
  TEST_ASSERT_FALSE(ramp_guard::rampDone(14999));
}

void test_rampDone_15000(void) {
  // t=15000 >= kTotalMs (15000)
  TEST_ASSERT_TRUE(ramp_guard::rampDone(15000));
}

// overspeedBound tests
void test_overspeedBound_324(void) {
  // 324 + 324/4 + 10 = 324 + 81 + 10 = 415
  TEST_ASSERT_EQUAL_INT32(415, ramp_guard::overspeedBound(324));
}

void test_overspeedBound_0(void) {
  // 0 + 0/4 + 10 = 10
  TEST_ASSERT_EQUAL_INT32(10, ramp_guard::overspeedBound(0));
}

// check() tests
void test_check_t100_present100_cap324(void) {
  // t=100, present=100, within bound 415: None
  ramp_guard::StepInput s = {100, false, true, true, 100, 324};
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::Abort::None), static_cast<int>(ramp_guard::check(s)));
}

void test_check_t100_present415_cap324(void) {
  // t=100, present=415, at bound: None
  ramp_guard::StepInput s = {100, false, true, true, 415, 324};
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::Abort::None), static_cast<int>(ramp_guard::check(s)));
}

void test_check_t100_present416_cap324(void) {
  // t=100, present=416 > bound 415: Overspeed
  ramp_guard::StepInput s = {100, false, true, true, 416, 324};
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::Abort::Overspeed), static_cast<int>(ramp_guard::check(s)));
}

void test_check_t100_present_minus415_cap324(void) {
  // t=100, present=-415, at negative bound: None
  ramp_guard::StepInput s = {100, false, true, true, -415, 324};
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::Abort::None), static_cast<int>(ramp_guard::check(s)));
}

void test_check_t100_present_minus416_cap324(void) {
  // t=100, present=-416 < -bound: Overspeed
  ramp_guard::StepInput s = {100, false, true, true, -416, 324};
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::Abort::Overspeed), static_cast<int>(ramp_guard::check(s)));
}

void test_check_t20000_cap324(void) {
  // t=20000 not > kHardTimeoutMs (20000), so within timeout
  ramp_guard::StepInput s = {20000, false, true, true, 100, 324};
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::Abort::None), static_cast<int>(ramp_guard::check(s)));
}

void test_check_t20001_cap324(void) {
  // t=20001 > kHardTimeoutMs (20000): Timeout
  ramp_guard::StepInput s = {20001, false, true, true, 100, 324};
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::Abort::Timeout), static_cast<int>(ramp_guard::check(s)));
}

void test_check_priority_serial_byte(void) {
  // serial_byte=true beats all other faults: Timeout, WriteFail, ReadFail, Overspeed all true
  ramp_guard::StepInput s = {20001, true, false, false, 500, 324};
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::Abort::SerialByte), static_cast<int>(ramp_guard::check(s)));
}

void test_check_priority_timeout_beats_writefail(void) {
  // Timeout beats WriteFail
  ramp_guard::StepInput s = {20001, false, false, true, 100, 324};
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::Abort::Timeout), static_cast<int>(ramp_guard::check(s)));
}

void test_check_priority_writefail_beats_readfail(void) {
  // WriteFail beats ReadFail
  ramp_guard::StepInput s = {100, false, false, false, 100, 324};
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::Abort::WriteFail), static_cast<int>(ramp_guard::check(s)));
}

void test_check_priority_readfail_beats_overspeed(void) {
  // ReadFail beats Overspeed
  ramp_guard::StepInput s = {100, false, true, false, 500, 324};
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::Abort::ReadFail), static_cast<int>(ramp_guard::check(s)));
}

// windowExpired tests
void test_windowExpired_59999_0_60000(void) {
  // (59999 - 0) = 59999 < 60000: false
  TEST_ASSERT_FALSE(ramp_guard::windowExpired(59999, 0, 60000));
}

void test_windowExpired_60000_0_60000(void) {
  // (60000 - 0) = 60000 >= 60000: true
  TEST_ASSERT_TRUE(ramp_guard::windowExpired(60000, 0, 60000));
}

void test_windowExpired_wrap_safe(void) {
  // now=100, start=0xFFFFFF00u (4294967040), window=60000
  // elapsed = (100 - 0xFFFFFF00u) wrapped = 0x00000100u = 256, 256 < 60000: false
  TEST_ASSERT_FALSE(ramp_guard::windowExpired(100, 0xFFFFFF00u, 60000));
}

void test_windowExpired_1000_0_60000(void) {
  // (1000 - 0) = 1000 < 60000: false
  TEST_ASSERT_FALSE(ramp_guard::windowExpired(1000, 0, 60000));
}

// GoParser tests - helper function
ramp_guard::GoResult feedString(ramp_guard::GoParser& p, const char* str) {
  ramp_guard::GoResult result = ramp_guard::GoResult::Pending;
  for (int i = 0; str[i]; i++) {
    result = p.feed(str[i]);
  }
  return result;
}

void test_goparser_GO_newline(void) {
  // "GO\n" -> Pending, Pending, Go
  ramp_guard::GoParser p;
  ramp_guard::GoResult r1 = p.feed('G');
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Pending), static_cast<int>(r1));

  ramp_guard::GoResult r2 = p.feed('O');
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Pending), static_cast<int>(r2));

  ramp_guard::GoResult r3 = p.feed('\n');
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Go), static_cast<int>(r3));
}

void test_goparser_GO_crlf(void) {
  // "GO\r\n" -> Pending, Pending, Go, Pending
  ramp_guard::GoParser p;
  ramp_guard::GoResult r1 = p.feed('G');
  ramp_guard::GoResult r2 = p.feed('O');
  ramp_guard::GoResult r3 = p.feed('\r');
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Pending), static_cast<int>(r1));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Pending), static_cast<int>(r2));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Go), static_cast<int>(r3));

  ramp_guard::GoResult r4 = p.feed('\n');
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Pending), static_cast<int>(r4));
}

void test_goparser_go_lowercase(void) {
  // "go\n" -> Reject (case-sensitive)
  ramp_guard::GoParser p;
  ramp_guard::GoResult result = feedString(p, "go\n");
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Reject), static_cast<int>(result));
}

void test_goparser_GOO(void) {
  // "GOO\n" -> Reject (too long before overflow)
  ramp_guard::GoParser p;
  ramp_guard::GoResult result = feedString(p, "GOO\n");
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Reject), static_cast<int>(result));
}

void test_goparser_GOOOO(void) {
  // "GOOOO\n" -> Reject (overflow)
  ramp_guard::GoParser p;
  ramp_guard::GoResult result = feedString(p, "GOOOO\n");
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Reject), static_cast<int>(result));
}

void test_goparser_G_only(void) {
  // "G\n" -> Reject (too short)
  ramp_guard::GoParser p;
  ramp_guard::GoResult result = feedString(p, "G\n");
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Reject), static_cast<int>(result));
}

void test_goparser_space_GO(void) {
  // " GO\n" -> Reject (leading space)
  ramp_guard::GoParser p;
  ramp_guard::GoResult result = feedString(p, " GO\n");
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Reject), static_cast<int>(result));
}

void test_goparser_blank_line(void) {
  // "\n" -> Pending (blank line)
  ramp_guard::GoParser p;
  ramp_guard::GoResult result = p.feed('\n');
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Pending), static_cast<int>(result));
}

void test_goparser_GO_no_newline(void) {
  // "GO" without newline -> Pending (no trigger)
  ramp_guard::GoParser p;
  ramp_guard::GoResult r1 = p.feed('G');
  ramp_guard::GoResult r2 = p.feed('O');
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Pending), static_cast<int>(r1));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Pending), static_cast<int>(r2));
}

void test_goparser_reject_then_accept(void) {
  // After a Reject line, a fresh "GO\n" gives Go
  ramp_guard::GoParser p;
  feedString(p, "go\n");  // Reject
  TEST_ASSERT_EQUAL_INT(p.n, 0);  // parser reset

  ramp_guard::GoResult result = feedString(p, "GO\n");
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ramp_guard::GoResult::Go), static_cast<int>(result));
}

// rampAllowed tests
void test_rampAllowed_all_ok(void) {
  // All fields true, limit_raw=1620, pct=20 -> true
  ramp_guard::Pre p = {true, true, true, true, true, true, true, 1620};
  TEST_ASSERT_TRUE(ramp_guard::rampAllowed(p, 20));
}

void test_rampAllowed_ping_ok_false(void) {
  ramp_guard::Pre p = {false, true, true, true, true, true, true, 1620};
  TEST_ASSERT_FALSE(ramp_guard::rampAllowed(p, 20));
}

void test_rampAllowed_model_known_false(void) {
  ramp_guard::Pre p = {true, false, true, true, true, true, true, 1620};
  TEST_ASSERT_FALSE(ramp_guard::rampAllowed(p, 20));
}

void test_rampAllowed_torque_off_confirmed_false(void) {
  ramp_guard::Pre p = {true, true, false, true, true, true, true, 1620};
  TEST_ASSERT_FALSE(ramp_guard::rampAllowed(p, 20));
}

void test_rampAllowed_limit_from_device_false(void) {
  ramp_guard::Pre p = {true, true, true, false, true, true, true, 1620};
  TEST_ASSERT_FALSE(ramp_guard::rampAllowed(p, 20));
}

void test_rampAllowed_mode_ok_false(void) {
  ramp_guard::Pre p = {true, true, true, true, false, true, true, 1620};
  TEST_ASSERT_FALSE(ramp_guard::rampAllowed(p, 20));
}

void test_rampAllowed_host_connected_false(void) {
  ramp_guard::Pre p = {true, true, true, true, true, false, true, 1620};
  TEST_ASSERT_FALSE(ramp_guard::rampAllowed(p, 20));
}

void test_rampAllowed_id_is_bench_false(void) {
  ramp_guard::Pre p = {true, true, true, true, true, true, false, 1620};
  TEST_ASSERT_FALSE(ramp_guard::rampAllowed(p, 20));
}

void test_rampAllowed_limit_raw_4(void) {
  // capFromLimit(4, 20) = 0, so rampAllowed returns false
  ramp_guard::Pre p = {true, true, true, true, true, true, true, 4};
  TEST_ASSERT_FALSE(ramp_guard::rampAllowed(p, 20));
}

void test_rampAllowed_limit_raw_0(void) {
  // capFromLimit(0, 20) = 0, so rampAllowed returns false
  ramp_guard::Pre p = {true, true, true, true, true, true, true, 0};
  TEST_ASSERT_FALSE(ramp_guard::rampAllowed(p, 20));
}

void test_rampAllowed_pct_26(void) {
  // capFromLimit(1620, 26) = 0, so rampAllowed returns false
  ramp_guard::Pre p = {true, true, true, true, true, true, true, 1620};
  TEST_ASSERT_FALSE(ramp_guard::rampAllowed(p, 26));
}

int main() {
  UNITY_BEGIN();

  // capFromLimit tests
  RUN_TEST(test_capFromLimit_basic_1620_20);
  RUN_TEST(test_capFromLimit_basic_2047_20);
  RUN_TEST(test_capFromLimit_basic_1620_25);
  RUN_TEST(test_capFromLimit_basic_100_20);
  RUN_TEST(test_capFromLimit_reject_4_20);
  RUN_TEST(test_capFromLimit_reject_1_20);
  RUN_TEST(test_capFromLimit_reject_0);
  RUN_TEST(test_capFromLimit_reject_negative);
  RUN_TEST(test_capFromLimit_reject_2048);
  RUN_TEST(test_capFromLimit_reject_pct_26);
  RUN_TEST(test_capFromLimit_reject_pct_0);

  // goalAt tests
  RUN_TEST(test_goalAt_t0_cap324);
  RUN_TEST(test_goalAt_t250_cap324);
  RUN_TEST(test_goalAt_t500_cap324);
  RUN_TEST(test_goalAt_t1000_cap324);
  RUN_TEST(test_goalAt_t3500_cap324);
  RUN_TEST(test_goalAt_t6999_cap324);
  RUN_TEST(test_goalAt_t7000_cap324);
  RUN_TEST(test_goalAt_t7999_cap324);
  RUN_TEST(test_goalAt_t8000_cap324);
  RUN_TEST(test_goalAt_t11500_cap324);
  RUN_TEST(test_goalAt_t14000_cap324);
  RUN_TEST(test_goalAt_t14999_cap324);
  RUN_TEST(test_goalAt_t15000_cap324);
  RUN_TEST(test_goalAt_t99999_cap324);
  RUN_TEST(test_goalAt_t3500_cap0);
  RUN_TEST(test_goalAt_monotonic_up);
  RUN_TEST(test_goalAt_monotonic_down);

  // rampDone tests
  RUN_TEST(test_rampDone_14999);
  RUN_TEST(test_rampDone_15000);

  // overspeedBound tests
  RUN_TEST(test_overspeedBound_324);
  RUN_TEST(test_overspeedBound_0);

  // check tests
  RUN_TEST(test_check_t100_present100_cap324);
  RUN_TEST(test_check_t100_present415_cap324);
  RUN_TEST(test_check_t100_present416_cap324);
  RUN_TEST(test_check_t100_present_minus415_cap324);
  RUN_TEST(test_check_t100_present_minus416_cap324);
  RUN_TEST(test_check_t20000_cap324);
  RUN_TEST(test_check_t20001_cap324);
  RUN_TEST(test_check_priority_serial_byte);
  RUN_TEST(test_check_priority_timeout_beats_writefail);
  RUN_TEST(test_check_priority_writefail_beats_readfail);
  RUN_TEST(test_check_priority_readfail_beats_overspeed);

  // windowExpired tests
  RUN_TEST(test_windowExpired_59999_0_60000);
  RUN_TEST(test_windowExpired_60000_0_60000);
  RUN_TEST(test_windowExpired_wrap_safe);
  RUN_TEST(test_windowExpired_1000_0_60000);

  // GoParser tests
  RUN_TEST(test_goparser_GO_newline);
  RUN_TEST(test_goparser_GO_crlf);
  RUN_TEST(test_goparser_go_lowercase);
  RUN_TEST(test_goparser_GOO);
  RUN_TEST(test_goparser_GOOOO);
  RUN_TEST(test_goparser_G_only);
  RUN_TEST(test_goparser_space_GO);
  RUN_TEST(test_goparser_blank_line);
  RUN_TEST(test_goparser_GO_no_newline);
  RUN_TEST(test_goparser_reject_then_accept);

  // rampAllowed tests
  RUN_TEST(test_rampAllowed_all_ok);
  RUN_TEST(test_rampAllowed_ping_ok_false);
  RUN_TEST(test_rampAllowed_model_known_false);
  RUN_TEST(test_rampAllowed_torque_off_confirmed_false);
  RUN_TEST(test_rampAllowed_limit_from_device_false);
  RUN_TEST(test_rampAllowed_mode_ok_false);
  RUN_TEST(test_rampAllowed_host_connected_false);
  RUN_TEST(test_rampAllowed_id_is_bench_false);
  RUN_TEST(test_rampAllowed_limit_raw_4);
  RUN_TEST(test_rampAllowed_limit_raw_0);
  RUN_TEST(test_rampAllowed_pct_26);

  return UNITY_END();
}
