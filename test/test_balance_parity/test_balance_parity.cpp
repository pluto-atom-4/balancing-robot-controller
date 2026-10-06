#include <unity.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <hal_iface.h>
#include <balance_core.h>
#include <balance_gains_generated.h>
#include "vectors_generated.inc"  // GENERATED, never edit; no include guard: include once only

// Parity of the C++ float32 core vs the Python float64 reference on host x86.
// NOT evidence about MCU timing or hardware. Gains/vectors are SIM-derived.
// On mismatch: fix the core (balancing-robot-controller#25) or the Python vectors via a
// new issue. NEVER loosen kTolAbs here (it comes from the generated .inc).
// PidStep.reset: call reset() BEFORE applying that step. dt<0 rows expect cmd 0, raw 0, fault true.
// Not covered by vectors (covered in test_balance_core only): NaN dt, out_limit <= 0.

namespace {

constexpr std::size_t kNLqr = sizeof(kLqrCases) / sizeof(kLqrCases[0]);
constexpr std::size_t kNPid = sizeof(kPidSeq) / sizeof(kPidSeq[0]);

static_assert(kNLqr >= 20, "need >= 20 LQR cases");
static_assert(kNPid >= 50, "need >= 50 PID steps");
static_assert(kPidSeq[0].reset, "PID sequence must start with reset");
static_assert(kTolAbs > 0.0f, "kTolAbs must be positive");

hal::ImuSample make_imu(float pitch, float gyro_y) {
  hal::ImuSample s{};
  s.pitch_rad = pitch;
  s.gyro_rad_s[1] = gyro_y;
  return s;
}
balance::LqrBalance make_lqr() {
  return balance::LqrBalance(balance_gains::kLqrK[0], balance_gains::kLqrK[1],
                             balance_gains::kThetaRefRad, balance_gains::kOutLimit);
}
balance::PidBalance make_pid() {
  return balance::PidBalance(balance_gains::kPidKp, balance_gains::kPidKi,
                             balance_gains::kPidKd, balance_gains::kPidOutLimit);
}

struct Tally {
  int fails;
  int first_idx;
  char first_msg[200];
};

bool within(float expected, float got) { return std::fabs(got - expected) <= kTolAbs; }

void note_first(Tally& t, int idx, const char* line) {
  std::printf("PARITY MISMATCH %s\n", line);
  if (t.fails == 0) {
    t.first_idx = idx;
    std::snprintf(t.first_msg, sizeof t.first_msg, "%s", line);
  }
  ++t.fails;
}

void check_float(Tally& t, const char* who, std::size_t idx, const char* field,
                 float expected, float got) {
  if (within(expected, got)) return;  // NaN fails here on purpose
  char line[200];
  std::snprintf(line, sizeof line, "%s[%d] %s expected %.9g got %.9g (tol %.9g)", who,
                static_cast<int>(idx), field, static_cast<double>(expected),
                static_cast<double>(got), static_cast<double>(kTolAbs));
  note_first(t, static_cast<int>(idx), line);
}

void check_row(Tally& t, const char* who, std::size_t idx, float exp_cmd, float exp_raw,
               bool exp_fault, const balance::ControlOutput& o) {
  if (o.fault != exp_fault) {
    char line[200];
    std::snprintf(line, sizeof line, "%s[%d] fault expected %d got %d", who,
                  static_cast<int>(idx), exp_fault ? 1 : 0, o.fault ? 1 : 0);
    note_first(t, static_cast<int>(idx), line);
  }
  check_float(t, who, idx, "cmd", exp_cmd, o.cmd_rad_s);
  check_float(t, who, idx, "raw", exp_raw, o.raw_cmd);
}

void finish(const Tally& t, const char* who) {
  char msg[400];
  std::snprintf(msg, sizeof msg,
                "%s parity: %d mismatch(es); FIRST: %s. Later PID rows may be knock-on. "
                "Do NOT loosen tolerance: fix core or open an issue.",
                who, t.fails, t.first_msg);
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, t.fails, msg);
}

}  // namespace

void setUp(void) {}
void tearDown(void) {}

void test_stamps_vectors_match_gains_header(void) {
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(balance_gains::kSchema, kVecSchema,
                                   "schema: vectors != gains header (copied at different times?)");
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(balance_gains::kContractVersion, kVecContractVersion,
                                   "contract: vectors != gains header");
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(hal::kContractVersion, kVecContractVersion,
                                   "contract: vectors != hal::kContractVersion");
  TEST_ASSERT_EQUAL_HEX32_MESSAGE(balance_gains::kGainsHash, kVecGainsHash,
                                  "gains hash: vectors != gains header; re-run export_cpp.py");
}

void test_vectors_shape(void) {
  int lqr_faults = 0, lqr_dt0 = 0, pid_resets = 0, pid_faults = 0, pid_dt0 = 0;
  for (std::size_t i = 0; i < kNLqr; ++i) {
    if (kLqrCases[i].fault) ++lqr_faults;
    if (kLqrCases[i].dt == 0.0f) ++lqr_dt0;
  }
  for (std::size_t i = 0; i < kNPid; ++i) {
    if (kPidSeq[i].reset) ++pid_resets;
    if (kPidSeq[i].fault) ++pid_faults;
    if (kPidSeq[i].dt == 0.0f) ++pid_dt0;
  }
  TEST_ASSERT_TRUE_MESSAGE(lqr_faults >= 2, "LQR vectors: need >= 2 fault rows");
  TEST_ASSERT_TRUE_MESSAGE(lqr_dt0 >= 1, "LQR vectors: need a dt==0 row");
  TEST_ASSERT_TRUE_MESSAGE(pid_resets >= 2, "PID vectors: need >= 2 reset rows");
  TEST_ASSERT_TRUE_MESSAGE(pid_faults >= 1, "PID vectors: need a fault row");
  TEST_ASSERT_TRUE_MESSAGE(pid_dt0 >= 1, "PID vectors: need a dt==0 row");
}

void test_lqr_matches_vectors(void) {
  const balance::LqrBalance lqr = make_lqr();
  Tally t = {};
  for (std::size_t i = 0; i < kNLqr; ++i) {
    const LqrCase& c = kLqrCases[i];
    const balance::ControlOutput o = lqr.step(make_imu(c.pitch, c.gyro_y), c.dt);
    check_row(t, "LQR", i, c.cmd, c.raw, c.fault, o);
  }
  finish(t, "LQR");
}

void test_pid_matches_vectors(void) {
  balance::PidBalance pid = make_pid();
  Tally t = {};
  for (std::size_t i = 0; i < kNPid; ++i) {
    const PidStep& s = kPidSeq[i];
    if (s.reset) pid.reset();  // reset BEFORE the step
    const balance::ControlOutput o = pid.step(make_imu(s.pitch, 0.0f), s.dt);
    check_row(t, "PID", i, s.cmd, s.raw, s.fault, o);
  }
  finish(t, "PID");
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_stamps_vectors_match_gains_header);
  RUN_TEST(test_vectors_shape);
  RUN_TEST(test_lqr_matches_vectors);
  RUN_TEST(test_pid_matches_vectors);
  return UNITY_END();
}
