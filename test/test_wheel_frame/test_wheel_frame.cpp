#include <unity.h>
#include <wheel_frame.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

void setUp(void) {}

void tearDown(void) {}

// Golden WHL lines, compared as literal strings.
static const char kG1[] = "WHL,1,20000,0.500000,21,0*54\n";
static const char kG2[] = "WHL,65535,4294967295,-1.250000,-125,2*5E\n";
static const char kG3[] = "WHL,0,0,0.000000,0,1*60\n";
static const char kG4[] = "WHL,7,140000,-0.000001,0,0*7F\n";

// Prefix of a zero frame with state 0 (checksum not compared here; see the checksum tests).
static const char kZeroPrefix[] = "WHL,0,0,0.000000,0,0*";

// Independent XOR over the body (everything before '*'), written apart from the library.
static uint8_t body_xor(const char* line) {
  uint8_t x = 0;
  for (size_t i = 0; line[i] != '\0' && line[i] != '*'; i++) {
    x = static_cast<uint8_t>(x ^ static_cast<uint8_t>(line[i]));
  }
  return x;
}

static void assert_hh_matches(const char* line) {
  const char* star = strchr(line, '*');
  TEST_ASSERT_NOT_NULL(star);
  char expect[3];
  snprintf(expect, sizeof expect, "%02X", static_cast<unsigned>(body_xor(line)));
  TEST_ASSERT_EQUAL_INT(0, strncmp(star + 1, expect, 2));
  TEST_ASSERT_EQUAL_CHAR('\n', star[3]);
}

// G1 golden, exact string and length
void test_golden_WHL_1(void) {
  char buf[64];
  size_t n = wheel_frame::format(buf, sizeof buf, 1, 20000u, 0.5f, 21, 0);
  TEST_ASSERT_EQUAL_UINT(29, n);
  TEST_ASSERT_EQUAL_STRING(kG1, buf);
}

// G2 golden, seq and t_us maxima, negative model and raw
void test_golden_WHL_2(void) {
  char buf[64];
  size_t n = wheel_frame::format(buf, sizeof buf, 65535, 4294967295u, -1.25f, -125, 2);
  TEST_ASSERT_EQUAL_UINT(41, n);
  TEST_ASSERT_EQUAL_STRING(kG2, buf);
}

// G3 golden, all zero with state 1
void test_golden_WHL_3(void) {
  char buf[64];
  size_t n = wheel_frame::format(buf, sizeof buf, 0, 0, 0.0f, 0, 1);
  TEST_ASSERT_EQUAL_UINT(24, n);
  TEST_ASSERT_EQUAL_STRING(kG3, buf);
}

// G4 golden, model -0.000001 keeps its sign
void test_golden_WHL_4(void) {
  char buf[64];
  size_t n = wheel_frame::format(buf, sizeof buf, 7, 140000u, -0.000001f, 0, 0);
  TEST_ASSERT_EQUAL_UINT(30, n);
  TEST_ASSERT_EQUAL_STRING(kG4, buf);
}

// checksum of "WHL," is 0x7F
void test_checksum_WHL_comma(void) {
  TEST_ASSERT_EQUAL_HEX8(0x7F, wheel_frame::checksum("WHL,", 4));
}

// checksum of a single 'W' is 0x57
void test_checksum_W_single(void) {
  TEST_ASSERT_EQUAL_HEX8(0x57, wheel_frame::checksum("W", 1));
}

// checksum of the G3 body (up to '*') is 0x60
void test_checksum_body_WHL_3(void) {
  const size_t body_len = static_cast<size_t>(strchr(kG3, '*') - kG3);
  TEST_ASSERT_EQUAL_UINT(20, body_len);
  TEST_ASSERT_EQUAL_HEX8(0x60, wheel_frame::checksum(kG3, body_len));
}

// library checksum, independent XOR and printed HH agree on all goldens
void test_checksum_matches_printed_hh(void) {
  const char* lines[4] = {kG1, kG2, kG3, kG4};
  for (int i = 0; i < 4; i++) {
    const size_t body_len = static_cast<size_t>(strchr(lines[i], '*') - lines[i]);
    TEST_ASSERT_EQUAL_HEX8(body_xor(lines[i]), wheel_frame::checksum(lines[i], body_len));
    assert_hh_matches(lines[i]);
  }
}

// field order is WHL, seq, t_us, model, raw, state
void test_field_order(void) {
  char buf[64];
  size_t n = wheel_frame::format(buf, sizeof buf, 5, 6, 0.0f, 7, 2);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_EQUAL_INT(0, strncmp(buf, "WHL,5,6,0.000000,7,2*", 21));
  assert_hh_matches(buf);
}

// model is printed with %.6f
void test_model_six_decimals(void) {
  char buf[64];
  wheel_frame::format(buf, sizeof buf, 0, 0, 0.25f, 0, 0);
  TEST_ASSERT_EQUAL_INT(0, strncmp(buf, "WHL,0,0,0.250000,0,0*", 21));
  wheel_frame::format(buf, sizeof buf, 0, 0, -2.5f, 0, 0);
  TEST_ASSERT_EQUAL_INT(0, strncmp(buf, "WHL,0,0,-2.500000,0,0*", 22));
}

// non-finite model (NaN, +inf, -inf) is refused in every state: returns 0,
// out[0] cleared, no frame written (bytes 1..63 keep the canary)
void test_nonfinite_model_rejected(void) {
  const float bad[3] = {NAN, INFINITY, -INFINITY};
  for (int i = 0; i < 3; i++) {
    for (uint8_t s = 0; s <= 2; s++) {
      unsigned char canary[64];
      memset(canary, 0x55, sizeof canary);
      char* buf = reinterpret_cast<char*>(canary);
      buf[0] = 'X';
      size_t n = wheel_frame::format(buf, sizeof canary, 0, 0, bad[i], 0, s);
      TEST_ASSERT_EQUAL_UINT(0, n);
      TEST_ASSERT_EQUAL_CHAR('\0', buf[0]);
      for (size_t k = 1; k < sizeof canary; k++) {
        TEST_ASSERT_EQUAL_HEX8(0x55, canary[k]);
      }
    }
  }
}

// Finite extremes and a tiny finite value still format (the non-finite check is not over-broad)
void test_finite_values_still_format(void) {
  char buf[64];
  size_t n = wheel_frame::format(buf, sizeof buf, 0, 0, 3.0f, 125, 0);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_EQUAL_INT(0, strncmp(buf, "WHL,0,0,3.000000,125,0*", strlen("WHL,0,0,3.000000,125,0*")));

  n = wheel_frame::format(buf, sizeof buf, 0, 0, -3.0f, -125, 0);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_EQUAL_INT(0, strncmp(buf, "WHL,0,0,-3.000000,-125,0*", strlen("WHL,0,0,-3.000000,-125,0*")));

  n = wheel_frame::format(buf, sizeof buf, 0, 0, 1e-9f, 0, 0);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_NOT_NULL(strstr(buf, ",0.000000,"));
}

// -0.0f and -1e-9f print as 0.000000 (no sign)
void test_negative_zero_and_tiny_negative_normalized(void) {
  const float near_zero[2] = {-0.0f, -1e-9f};
  char buf[64];
  for (int i = 0; i < 2; i++) {
    size_t n = wheel_frame::format(buf, sizeof buf, 0, 0, near_zero[i], 0, 0);
    TEST_ASSERT_EQUAL_UINT(24, n);
    TEST_ASSERT_EQUAL_INT(0, strncmp(buf, kZeroPrefix, strlen(kZeroPrefix)));
  }
}

// negative raw is printed with its sign, INT32_MIN included
void test_negative_raw(void) {
  char buf[64];
  size_t n = wheel_frame::format(buf, sizeof buf, 0, 0, 0.0f, INT32_MIN, 0);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_EQUAL_INT(0, strncmp(buf, "WHL,0,0,0.000000,-2147483648,0*", 31));
  assert_hh_matches(buf);
}

// states 0, 1 and 2 are accepted and printed as the last field
void test_states_0_1_2_accepted(void) {
  char buf[64];
  for (int s = 0; s <= 2; s++) {
    size_t n = wheel_frame::format(buf, sizeof buf, 0, 0, 0.0f, 0, static_cast<uint8_t>(s));
    TEST_ASSERT_EQUAL_UINT(24, n);
    const char* star = strchr(buf, '*');
    TEST_ASSERT_NOT_NULL(star);
    TEST_ASSERT_EQUAL_CHAR(static_cast<char>('0' + s), star[-1]);
  }
}

// States 3 and 255 are rejected, out[0] cleared
void test_state_3_rejected(void) {
  char buf[64];
  buf[0] = 'X';
  size_t n = wheel_frame::format(buf, sizeof buf, 0, 0, 0.0f, 0, 3);
  TEST_ASSERT_EQUAL_UINT(0, n);
  TEST_ASSERT_EQUAL_CHAR('\0', buf[0]);

  buf[0] = 'X';
  n = wheel_frame::format(buf, sizeof buf, 0, 0, 0.0f, 0, 255);
  TEST_ASSERT_EQUAL_UINT(0, n);
  TEST_ASSERT_EQUAL_CHAR('\0', buf[0]);
}

// exact-fit cap (frame + NUL) succeeds
void test_exact_fit_cap(void) {
  char buf[30];
  size_t n = wheel_frame::format(buf, sizeof buf, 1, 20000u, 0.5f, 21, 0);
  TEST_ASSERT_EQUAL_UINT(29, n);
  TEST_ASSERT_EQUAL_STRING(kG1, buf);
}

// Caps 0..29 are too small for the 30-byte golden 1 frame (NUL included): return 0,
// out[0] cleared when cap > 0, bytes at index >= cap untouched. Cap 30 returns the golden.
void test_cap_sweep_golden_1(void) {
  for (size_t cap = 0; cap < 30; cap++) {
    unsigned char canary[64];
    memset(canary, 0x55, sizeof canary);
    size_t n = wheel_frame::format(reinterpret_cast<char*>(canary), cap, 1, 20000u, 0.5f, 21, 0);
    TEST_ASSERT_EQUAL_UINT(0, n);
    if (cap > 0) {
      TEST_ASSERT_EQUAL_CHAR('\0', static_cast<char>(canary[0]));
    }
    for (size_t i = cap; i < sizeof canary; i++) {
      TEST_ASSERT_EQUAL_HEX8(0x55, canary[i]);
    }
  }
  char buf[64];
  memset(buf, 0x55, sizeof buf);
  size_t n = wheel_frame::format(buf, 30, 1, 20000u, 0.5f, 21, 0);
  TEST_ASSERT_EQUAL_UINT(29, n);
  TEST_ASSERT_EQUAL_STRING(kG1, buf);
}

// cap 0 and nullptr out return 0 and write nothing
void test_cap_zero_and_null_out(void) {
  unsigned char buf[4] = {0x55, 0x55, 0x55, 0x55};
  size_t n = wheel_frame::format(reinterpret_cast<char*>(buf), 0, 0, 0, 0.0f, 0, 0);
  TEST_ASSERT_EQUAL_UINT(0, n);
  TEST_ASSERT_EQUAL_HEX8(0x55, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x55, buf[3]);

  n = wheel_frame::format(nullptr, 100, 0, 0, 0.0f, 0, 0);
  TEST_ASSERT_EQUAL_UINT(0, n);
}

// max seq 65535 and max t_us 4294967295 with a zero model
void test_max_seq_and_t_us(void) {
  char buf[64];
  size_t n = wheel_frame::format(buf, sizeof buf, 65535, 4294967295u, 0.0f, 0, 0);
  TEST_ASSERT_EQUAL_UINT(37, n);
  TEST_ASSERT_EQUAL_INT(0, strncmp(buf, "WHL,65535,4294967295,0.000000,0,0*", 34));
  assert_hh_matches(buf);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_golden_WHL_1);
  RUN_TEST(test_golden_WHL_2);
  RUN_TEST(test_golden_WHL_3);
  RUN_TEST(test_golden_WHL_4);
  RUN_TEST(test_checksum_WHL_comma);
  RUN_TEST(test_checksum_W_single);
  RUN_TEST(test_checksum_body_WHL_3);
  RUN_TEST(test_checksum_matches_printed_hh);
  RUN_TEST(test_field_order);
  RUN_TEST(test_model_six_decimals);
  RUN_TEST(test_nonfinite_model_rejected);
  RUN_TEST(test_finite_values_still_format);
  RUN_TEST(test_negative_zero_and_tiny_negative_normalized);
  RUN_TEST(test_negative_raw);
  RUN_TEST(test_states_0_1_2_accepted);
  RUN_TEST(test_state_3_rejected);
  RUN_TEST(test_exact_fit_cap);
  RUN_TEST(test_cap_sweep_golden_1);
  RUN_TEST(test_cap_zero_and_null_out);
  RUN_TEST(test_max_seq_and_t_us);
  return UNITY_END();
}
