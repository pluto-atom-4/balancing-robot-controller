#include <unity.h>
#include <segway_cmd.h>
#include <cstring>

void setUp(void) {}

void tearDown(void) {}

struct FeedResult {
  unsigned lines;
  float last;
};

// Feeds every character of s and counts the complete valid lines.
static FeedResult feedStr(segway_cmd::LineParser& p, const char* s) {
  FeedResult r;
  r.lines = 0;
  r.last = 0.0f;
  for (const char* c = s; *c != '\0'; c++) {
    float v = 0.0f;
    if (p.feed(*c, &v)) {
      r.lines++;
      r.last = v;
    }
  }
  return r;
}

// A line that must be rejected: no result, rejected +1, accepted +0.
static void expectRejected(const char* s) {
  segway_cmd::LineParser p;
  FeedResult r = feedStr(p, s);
  TEST_ASSERT_EQUAL_UINT(0, r.lines);
  TEST_ASSERT_EQUAL_UINT32(1, p.rejected);
  TEST_ASSERT_EQUAL_UINT32(0, p.accepted);
}

void test_half(void) {
  segway_cmd::LineParser p;
  FeedResult r = feedStr(p, "CMD 0.5\n");
  TEST_ASSERT_EQUAL_UINT(1, r.lines);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, r.last);
  TEST_ASSERT_EQUAL_UINT32(1, p.accepted);
  TEST_ASSERT_EQUAL_UINT32(0, p.rejected);
}

void test_negative_quarters(void) {
  segway_cmd::LineParser p;
  FeedResult r = feedStr(p, "CMD -1.25\n");
  TEST_ASSERT_EQUAL_UINT(1, r.lines);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.25f, r.last);
}

void test_crlf_gives_one_result(void) {
  segway_cmd::LineParser p;
  FeedResult r = feedStr(p, "CMD 2.75\r\n");
  TEST_ASSERT_EQUAL_UINT(1, r.lines);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.75f, r.last);
  TEST_ASSERT_EQUAL_UINT32(1, p.accepted);
  TEST_ASSERT_EQUAL_UINT32(0, p.rejected);
}

void test_clamp_high(void) {
  segway_cmd::LineParser p;
  FeedResult r = feedStr(p, "CMD 7.0\n");
  TEST_ASSERT_EQUAL_UINT(1, r.lines);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f, r.last);
}

void test_clamp_low(void) {
  segway_cmd::LineParser p;
  FeedResult r = feedStr(p, "CMD -9\n");
  TEST_ASSERT_EQUAL_UINT(1, r.lines);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -3.0f, r.last);
}

void test_plus_sign(void) {
  segway_cmd::LineParser p;
  FeedResult r = feedStr(p, "CMD +1\n");
  TEST_ASSERT_EQUAL_UINT(1, r.lines);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, r.last);
}

void test_zero(void) {
  segway_cmd::LineParser p;
  FeedResult r = feedStr(p, "CMD 0\n");
  TEST_ASSERT_EQUAL_UINT(1, r.lines);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, r.last);
}

void test_reject_nan(void) {
  expectRejected("CMD nan\n");
}

void test_reject_inf(void) {
  expectRejected("CMD inf\n");
}

void test_reject_bare_cmd(void) {
  expectRejected("CMD\n");
}

void test_reject_letter(void) {
  expectRejected("CMD x\n");
}

void test_reject_exponent(void) {
  expectRejected("CMD 1e3\n");
}

void test_reject_two_dots(void) {
  expectRejected("CMD 1.2.3\n");
}

void test_reject_lone_sign(void) {
  expectRejected("CMD -\n");
}

void test_reject_wrong_prefix(void) {
  expectRejected("FOO 1\n");
}

void test_reject_lowercase_prefix(void) {
  expectRejected("cmd 1\n");
}

void test_reject_trailing_space(void) {
  expectRejected("CMD 1 \n");
}

void test_reject_long_line(void) {
  // "CMD " + 36 digits = 40 characters, longer than kLineMax - 1.
  char line[48];
  std::memcpy(line, "CMD ", 4);
  for (int i = 4; i < 40; i++) line[i] = '1';
  line[40] = '\n';
  line[41] = '\0';
  expectRejected(line);
}

void test_empty_lines_ignored(void) {
  segway_cmd::LineParser p;
  FeedResult r = feedStr(p, "\n\r\n\r");
  TEST_ASSERT_EQUAL_UINT(0, r.lines);
  TEST_ASSERT_EQUAL_UINT32(0, p.accepted);
  TEST_ASSERT_EQUAL_UINT32(0, p.rejected);
}

void test_recovery_after_bad_line(void) {
  segway_cmd::LineParser p;
  FeedResult r = feedStr(p, "CMD x\nCMD 1.5\n");
  TEST_ASSERT_EQUAL_UINT(1, r.lines);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, r.last);
  TEST_ASSERT_EQUAL_UINT32(1, p.accepted);
  TEST_ASSERT_EQUAL_UINT32(1, p.rejected);
}

void test_two_lines_in_order(void) {
  segway_cmd::LineParser p;
  const char* s = "CMD 0.5\nCMD -1.25\n";
  float got[2] = {99.0f, 99.0f};
  unsigned count = 0;
  for (const char* c = s; *c != '\0'; c++) {
    float v = 0.0f;
    if (p.feed(*c, &v) && count < 2) {
      got[count] = v;
      count++;
    }
  }
  TEST_ASSERT_EQUAL_UINT(2, count);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, got[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.25f, got[1]);
  TEST_ASSERT_EQUAL_UINT32(2, p.accepted);
  TEST_ASSERT_EQUAL_UINT32(0, p.rejected);
}

void test_out_untouched_on_reject(void) {
  segway_cmd::LineParser p;
  float out = 99.0f;
  const char* s = "CMD nan\n";
  for (const char* c = s; *c != '\0'; c++) {
    TEST_ASSERT_FALSE(p.feed(*c, &out));
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 99.0f, out);
  TEST_ASSERT_EQUAL_UINT32(1, p.rejected);
}

void test_overflow_then_good_line(void) {
  // The 40-character line is dropped until its terminator, then CMD 1 is read.
  char line[48];
  std::memcpy(line, "CMD ", 4);
  for (int i = 4; i < 40; i++) line[i] = '1';
  line[40] = '\n';
  line[41] = '\0';
  segway_cmd::LineParser p;
  FeedResult r = feedStr(p, line);
  TEST_ASSERT_EQUAL_UINT(0, r.lines);
  r = feedStr(p, "CMD 1\n");
  TEST_ASSERT_EQUAL_UINT(1, r.lines);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, r.last);
  TEST_ASSERT_EQUAL_UINT32(1, p.accepted);
  TEST_ASSERT_EQUAL_UINT32(1, p.rejected);
}

void test_reset_clears_counters(void) {
  segway_cmd::LineParser p;
  feedStr(p, "CMD x\nCMD 1\n");
  p.reset();
  TEST_ASSERT_EQUAL_UINT32(0, p.accepted);
  TEST_ASSERT_EQUAL_UINT32(0, p.rejected);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_half);
  RUN_TEST(test_negative_quarters);
  RUN_TEST(test_crlf_gives_one_result);
  RUN_TEST(test_clamp_high);
  RUN_TEST(test_clamp_low);
  RUN_TEST(test_plus_sign);
  RUN_TEST(test_zero);
  RUN_TEST(test_reject_nan);
  RUN_TEST(test_reject_inf);
  RUN_TEST(test_reject_bare_cmd);
  RUN_TEST(test_reject_letter);
  RUN_TEST(test_reject_exponent);
  RUN_TEST(test_reject_two_dots);
  RUN_TEST(test_reject_lone_sign);
  RUN_TEST(test_reject_wrong_prefix);
  RUN_TEST(test_reject_lowercase_prefix);
  RUN_TEST(test_reject_trailing_space);
  RUN_TEST(test_reject_long_line);
  RUN_TEST(test_empty_lines_ignored);
  RUN_TEST(test_recovery_after_bad_line);
  RUN_TEST(test_two_lines_in_order);
  RUN_TEST(test_out_untouched_on_reject);
  RUN_TEST(test_overflow_then_good_line);
  RUN_TEST(test_reset_clears_counters);
  return UNITY_END();
}
