// WHL2 formatter tests, B02 (#271); the header is B03 (#272); umbrella #267.
// SIM-ONLY, UNVERIFIED.
//
// Golden lines: the 11 accept lines of freecad-workspace
// poc/ctrl-wheels/CONTRACT.md S7 (literals compared with S7 on freecad-workspace
// origin/main 9ea88a4; the two sets are equal). This test only shows that the formatter
// reproduces them; each line's checksum is also recomputed here by body_xor.
// The reject lines in S7 are parser territory (the WHL2 parser, freecad-workspace
// ctrl_wheels_msg.py, PR #511) and are not used here.
//
// Frame: WHL2,<seq>,<t_us>,<model as %.6f>,<rawL>,<rawR>,<state>*HH\n
//   rawL and rawR are the raw values as passed in (the firmware will pass the
//   values it writes to each chip, task B12). The formatter does no sign logic
//   and no clamping, and these tests do not claim that any sign is correct (the
//   wheel signs are unconfirmed).
//   state: 0 run, 1 hold after a fall, 2 bus down; 3 and above are refused.
//   HH: 2 uppercase hex digits, XOR of the bytes from 'W' up to, not including, '*'.

#include <unity.h>
#include <wheels_frame.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

void setUp(void) {}

void tearDown(void) {}

static const char kLine1[] = "WHL2,1,1000000,0.500000,10,12,0*48\n";

struct GoldenCase {
  const char* name;
  uint16_t seq;
  uint32_t t_us;
  float model;
  int32_t rawL;
  int32_t rawR;
  uint8_t state;
  const char* line;
};

static const GoldenCase kGolden[] = {
  {"positive, all numeric values positive (format case, not a sign claim)", 1, 1000000u, 0.5f, 10, 12, 0, kLine1},
  {"rawL positive, rawR negative", 2, 1020000u, 0.5f, 10, -10, 0, "WHL2,2,1020000,0.500000,10,-10,0*66\n"},
  {"both raws negative", 3, 1040000u, -0.5f, -10, -10, 0, "WHL2,3,1040000,-0.500000,-10,-10,0*61\n"},
  {"-0 model (prints 0.000000)", 4, 1060000u, -0.0f, 0, 0, 0, "WHL2,4,1060000,0.000000,0,0,0*4C\n"},
  {"state 0", 5, 1080000u, 1.25f, 20, -20, 0, "WHL2,5,1080000,1.250000,20,-20,0*68\n"},
  {"state 1", 6, 1100000u, 0.0f, 0, 0, 1, "WHL2,6,1100000,0.000000,0,0,1*48\n"},
  {"state 2", 7, 1120000u, 0.0f, 0, 0, 2, "WHL2,7,1120000,0.000000,0,0,2*48\n"},
  {"checksum with an uppercase hex letter in HH", 8, 1140000u, 0.75f, 15, -15, 0, "WHL2,8,1140000,0.750000,15,-15,0*6C\n"},
  {"seq 65535", 65535, 1160000u, 0.25f, 5, -5, 0, "WHL2,65535,1160000,0.250000,5,-5,0*63\n"},
  {"t_us 4294967295", 9, 4294967295u, 0.25f, 5, -5, 0, "WHL2,9,4294967295,0.250000,5,-5,0*51\n"},
  {"int32 extremes (rawL INT32_MIN, rawR 2147483647)", 10, 1180000u, 1.0f, INT32_MIN, 2147483647, 0, "WHL2,10,1180000,1.000000,-2147483648,2147483647,0*55\n"},
};

static const size_t kGoldenCount = sizeof kGolden / sizeof kGolden[0];

// Independent XOR over the body (everything before '*'), written apart from the library.
static uint8_t body_xor(const char* line) {
  uint8_t x = 0;
  for (size_t i = 0; line[i] != '\0' && line[i] != '*'; i++) {
    x = static_cast<uint8_t>(x ^ static_cast<uint8_t>(line[i]));
  }
  return x;
}

// Checks that the frame ends with "*HH\n" and HH is the independent XOR of the body.
static void assert_hh_matches(const char* line) {
  const char* star = strchr(line, '*');
  TEST_ASSERT_NOT_NULL(star);
  char expect[3];
  snprintf(expect, sizeof expect, "%02X", static_cast<unsigned>(body_xor(line)));
  TEST_ASSERT_EQUAL_INT(0, strncmp(star + 1, expect, 2));
  TEST_ASSERT_EQUAL_CHAR('\n', star[3]);
  TEST_ASSERT_EQUAL_CHAR('\0', star[4]);
}

static void assert_prefix(const char* buf, const char* prefix) {
  TEST_ASSERT_EQUAL_INT(0, strncmp(buf, prefix, strlen(prefix)));
}

// Golden check: returned length equals strlen(literal), and the bytes match exactly.
static void expect_golden(const GoldenCase& g) {
  char msg[160];
  snprintf(msg, sizeof msg, "golden case failed: %s", g.name);
  char buf[96];
  size_t n = wheels_frame::format(buf, sizeof buf, g.seq, g.t_us, g.model, g.rawL, g.rawR, g.state);
  TEST_ASSERT_EQUAL_UINT_MESSAGE(strlen(g.line), n, msg);
  TEST_ASSERT_EQUAL_STRING_MESSAGE(g.line, buf, msg);
}

// Every accept golden matches exactly, with the length taken from strlen.
void test_golden_accept_cases(void) {
  for (size_t i = 0; i < kGoldenCount; i++) {
    expect_golden(kGolden[i]);
  }
}

// checksum of "WHL2," equals the independent XOR (0x4D, computed by hand from
// 'W' 'H' 'L' '2' ',' as well).
void test_checksum_WHL2_comma(void) {
  TEST_ASSERT_EQUAL_HEX8(body_xor("WHL2,"), wheels_frame::checksum("WHL2,", 5));
  TEST_ASSERT_EQUAL_HEX8(0x4D, wheels_frame::checksum("WHL2,", 5));
}

// On every golden: library checksum() equals the independent body_xor.
void test_checksum_library_matches_body_xor_on_goldens(void) {
  for (size_t i = 0; i < kGoldenCount; i++) {
    const char* line = kGolden[i].line;
    const char* star = strchr(line, '*');
    TEST_ASSERT_NOT_NULL(star);
    const size_t body_len = static_cast<size_t>(star - line);
    char msg[160];
    snprintf(msg, sizeof msg, "checksum mismatch: %s", kGolden[i].name);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(body_xor(line), wheels_frame::checksum(line, body_len), msg);
  }
}

// rawL and rawR are not swapped and keep their own signs.
void test_rawL_rawR_not_swapped(void) {
  char buf[64];
  size_t n = wheels_frame::format(buf, sizeof buf, 0, 0, 0.0f, 10, -10, 0);
  TEST_ASSERT_TRUE(n > 0);
  assert_prefix(buf, "WHL2,0,0,0.000000,10,-10,0*");
  assert_hh_matches(buf);
  n = wheels_frame::format(buf, sizeof buf, 0, 0, 0.0f, -10, 10, 0);
  TEST_ASSERT_TRUE(n > 0);
  assert_prefix(buf, "WHL2,0,0,0.000000,-10,10,0*");
  assert_hh_matches(buf);
}

// A non-finite model (NaN, +inf, -inf) is refused in every state 0..2: returns 0,
// out[0] cleared, nothing written at or after out[cap] (cap 32 of a 64-byte canary).
void test_nonfinite_model_rejected_all_states(void) {
  const float bad[3] = {NAN, INFINITY, -INFINITY};
  for (int i = 0; i < 3; i++) {
    for (uint8_t s = 0; s <= 2; s++) {
      unsigned char canary[64];
      memset(canary, 0x55, sizeof canary);
      char* buf = reinterpret_cast<char*>(canary);
      buf[0] = 'X';
      const size_t cap = 32;
      size_t n = wheels_frame::format(buf, cap, 0, 0, bad[i], 0, 0, s);
      TEST_ASSERT_EQUAL_UINT_MESSAGE(0, n, "non-finite model must return 0");
      TEST_ASSERT_EQUAL_CHAR_MESSAGE('\0', buf[0], "non-finite model must clear out[0]");
      for (size_t k = cap; k < sizeof canary; k++) {
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x55, canary[k], "non-finite model wrote at or after out[cap]");
      }
    }
  }
}

// model is printed with %.6f. Finite values still format (the non-finite check
// is not over-broad).
void test_finite_values_still_format(void) {
  char buf[64];
  size_t n = wheels_frame::format(buf, sizeof buf, 0, 0, 0.25f, 0, 0, 0);
  TEST_ASSERT_TRUE(n > 0);
  assert_prefix(buf, "WHL2,0,0,0.250000,0,0,0*");
  assert_hh_matches(buf);

  n = wheels_frame::format(buf, sizeof buf, 0, 0, -2.5f, 0, 0, 0);
  TEST_ASSERT_TRUE(n > 0);
  assert_prefix(buf, "WHL2,0,0,-2.500000,0,0,0*");
  assert_hh_matches(buf);

  n = wheels_frame::format(buf, sizeof buf, 0, 0, 1.0f, 0, 0, 0);
  TEST_ASSERT_TRUE(n > 0);
  assert_prefix(buf, "WHL2,0,0,1.000000,0,0,0*");
  assert_hh_matches(buf);

  n = wheels_frame::format(buf, sizeof buf, 0, 0, 3.0f, 125, 125, 0);
  TEST_ASSERT_TRUE(n > 0);
  assert_prefix(buf, "WHL2,0,0,3.000000,125,125,0*");
  assert_hh_matches(buf);

  n = wheels_frame::format(buf, sizeof buf, 0, 0, -3.0f, -125, -125, 0);
  TEST_ASSERT_TRUE(n > 0);
  assert_prefix(buf, "WHL2,0,0,-3.000000,-125,-125,0*");
  assert_hh_matches(buf);

  n = wheels_frame::format(buf, sizeof buf, 0, 0, 1e-9f, 0, 0, 0);
  TEST_ASSERT_TRUE(n > 0);
  assert_prefix(buf, "WHL2,0,0,0.000000,0,0,0*");
  assert_hh_matches(buf);
}

// -0.0f and -1e-9f print as 0.000000 with no sign anywhere in the frame.
void test_negative_zero_and_tiny_negative_normalized(void) {
  const float near_zero[2] = {-0.0f, -1e-9f};
  char buf[64];
  for (int i = 0; i < 2; i++) {
    size_t n = wheels_frame::format(buf, sizeof buf, 0, 0, near_zero[i], 0, 0, 0);
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_EQUAL_UINT(strlen(buf), n);
    assert_prefix(buf, "WHL2,0,0,0.000000,0,0,0*");
    TEST_ASSERT_NULL(strchr(buf, '-'));
    assert_hh_matches(buf);
  }
}

// INT32_MIN and INT32_MAX in both raw fields, printed as signed decimals.
void test_int32_extremes_both_raw_fields(void) {
  char buf[64];
  size_t n = wheels_frame::format(buf, sizeof buf, 0, 0, 0.0f, INT32_MIN, 2147483647, 0);
  TEST_ASSERT_TRUE(n > 0);
  assert_prefix(buf, "WHL2,0,0,0.000000,-2147483648,2147483647,0*");
  assert_hh_matches(buf);

  n = wheels_frame::format(buf, sizeof buf, 0, 0, 0.0f, 2147483647, INT32_MIN, 0);
  TEST_ASSERT_TRUE(n > 0);
  assert_prefix(buf, "WHL2,0,0,0.000000,2147483647,-2147483648,0*");
  assert_hh_matches(buf);
}

// States 3 and 255 are refused: returns 0, out[0] cleared, nothing at or after out[cap].
void test_state_3_and_255_rejected(void) {
  const uint8_t bad[2] = {3, 255};
  for (int i = 0; i < 2; i++) {
    unsigned char canary[64];
    memset(canary, 0x55, sizeof canary);
    char* buf = reinterpret_cast<char*>(canary);
    buf[0] = 'X';
    const size_t cap = 32;
    size_t n = wheels_frame::format(buf, cap, 0, 0, 0.0f, 0, 0, bad[i]);
    TEST_ASSERT_EQUAL_UINT(0, n);
    TEST_ASSERT_EQUAL_CHAR('\0', buf[0]);
    for (size_t k = cap; k < sizeof canary; k++) {
      TEST_ASSERT_EQUAL_HEX8(0x55, canary[k]);
    }
  }
}

// Caps 0..len (len = golden 1 frame length, NUL not counted) are refused, with
// out[0] cleared when cap > 0 and nothing at or after out[cap]. Cap len + 1 succeeds.
void test_cap_sweep_golden_1_canary(void) {
  const size_t len = sizeof kLine1 - 1;
  for (size_t cap = 0; cap <= len; cap++) {
    unsigned char canary[64];
    memset(canary, 0x55, sizeof canary);
    char* buf = reinterpret_cast<char*>(canary);
    size_t n = wheels_frame::format(buf, cap, 1, 1000000u, 0.5f, 10, 12, 0);
    TEST_ASSERT_EQUAL_UINT(0, n);
    if (cap > 0) {
      TEST_ASSERT_EQUAL_CHAR('\0', buf[0]);
    }
    for (size_t k = cap; k < sizeof canary; k++) {
      TEST_ASSERT_EQUAL_HEX8(0x55, canary[k]);
    }
  }
  char buf[64];
  memset(buf, 0x55, sizeof buf);
  size_t n = wheels_frame::format(buf, len + 1, 1, 1000000u, 0.5f, 10, 12, 0);
  TEST_ASSERT_EQUAL_UINT(len, n);
  TEST_ASSERT_EQUAL_STRING(kLine1, buf);
}

// cap 0 and nullptr out return 0 and write nothing.
void test_cap_zero_and_null_out(void) {
  unsigned char buf[4] = {0x55, 0x55, 0x55, 0x55};
  size_t n = wheels_frame::format(reinterpret_cast<char*>(buf), 0, 0, 0, 0.0f, 0, 0, 0);
  TEST_ASSERT_EQUAL_UINT(0, n);
  for (size_t k = 0; k < sizeof buf; k++) {
    TEST_ASSERT_EQUAL_HEX8(0x55, buf[k]);
  }

  n = wheels_frame::format(nullptr, 100, 0, 0, 0.0f, 0, 0, 0);
  TEST_ASSERT_EQUAL_UINT(0, n);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_golden_accept_cases);
  RUN_TEST(test_checksum_WHL2_comma);
  RUN_TEST(test_checksum_library_matches_body_xor_on_goldens);
  RUN_TEST(test_rawL_rawR_not_swapped);
  RUN_TEST(test_finite_values_still_format);
  RUN_TEST(test_nonfinite_model_rejected_all_states);
  RUN_TEST(test_negative_zero_and_tiny_negative_normalized);
  RUN_TEST(test_int32_extremes_both_raw_fields);
  RUN_TEST(test_state_3_and_255_rejected);
  RUN_TEST(test_cap_sweep_golden_1_canary);
  RUN_TEST(test_cap_zero_and_null_out);
  return UNITY_END();
}
