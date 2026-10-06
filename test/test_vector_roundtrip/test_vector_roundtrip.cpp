#include <unity.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <hal_iface.h>
#include <balance_core.h>
#include <balance_gains_generated.h>
#include <balance_stamp.h>
#include "../test_balance_parity/vectors_generated.inc"  // GENERATED, never edit; include once only

// Round trip of the committed Python-exported gains + vectors, plus negative cases for
// the stamp check and a discrimination check (a corrupted gain MUST be caught).
// Host x86 float32 only. NOT evidence about MCU timing or hardware. SIM-derived values.
// Parity itself is owned by test_balance_parity (#29); the control test here exists only
// so the discrimination tests are not vacuous. NEVER loosen kTolAbs (it comes from the .inc).

namespace {

using balance_stamp::Result;
using balance_stamp::Stamp;

constexpr std::size_t kNLqr = sizeof(kLqrCases) / sizeof(kLqrCases[0]);
constexpr std::size_t kNPid = sizeof(kPidSeq) / sizeof(kPidSeq[0]);

// Same pin as test_balance_stamp (schema 1). Bump both on purpose.
constexpr uint32_t kExpectSchema = 1;
constexpr uint32_t kExpectContract = hal::kContractVersion;

constexpr Stamp real_gains() {
  return Stamp{balance_gains::kSchema, balance_gains::kContractVersion,
               balance_gains::kGainsHash};
}
constexpr Stamp real_vecs() { return Stamp{kVecSchema, kVecContractVersion, kVecGainsHash}; }

void expect_result(Result want, Result got) {
  char msg[80];
  std::snprintf(msg, sizeof msg, "want %s got %s", balance_stamp::name(want),
                balance_stamp::name(got));
  TEST_ASSERT_EQUAL_INT_MESSAGE(static_cast<int>(want), static_cast<int>(got), msg);
}

Result run_check(const Stamp& g, const Stamp& v) {
  return balance_stamp::check(g, v, kExpectSchema, kExpectContract);
}

hal::ImuSample make_imu(float pitch, float gyro_y) {
  hal::ImuSample s{};
  s.pitch_rad = pitch;
  s.gyro_rad_s[1] = gyro_y;
  return s;
}

struct Exceed {
  int cmd;  // rows where |cmd - expected| > kTolAbs (or NaN)
  int raw;  // rows where |raw - expected| > kTolAbs (or NaN)
};

bool off(float expected, float got) { return !(std::fabs(got - expected) <= kTolAbs); }

Exceed lqr_exceed(const balance::LqrBalance& lqr) {
  Exceed e{0, 0};
  for (std::size_t i = 0; i < kNLqr; ++i) {
    const LqrCase& c = kLqrCases[i];
    const balance::ControlOutput o = lqr.step(make_imu(c.pitch, c.gyro_y), c.dt);
    if (off(c.cmd, o.cmd_rad_s)) ++e.cmd;
    if (off(c.raw, o.raw_cmd)) ++e.raw;
  }
  return e;
}

Exceed pid_exceed(balance::PidBalance pid) {
  Exceed e{0, 0};
  for (std::size_t i = 0; i < kNPid; ++i) {
    const PidStep& s = kPidSeq[i];
    if (s.reset) pid.reset();  // reset BEFORE the step
    const balance::ControlOutput o = pid.step(make_imu(s.pitch, 0.0f), s.dt);
    if (off(s.cmd, o.cmd_rad_s)) ++e.cmd;
    if (off(s.raw, o.raw_cmd)) ++e.raw;
  }
  return e;
}

balance::LqrBalance lqr_with_k0_scale(float scale) {
  return balance::LqrBalance(balance_gains::kLqrK[0] * scale, balance_gains::kLqrK[1],
                             balance_gains::kThetaRefRad, balance_gains::kOutLimit);
}
balance::PidBalance pid_with_kp_scale(float scale) {
  return balance::PidBalance(balance_gains::kPidKp * scale, balance_gains::kPidKi,
                             balance_gains::kPidKd, balance_gains::kPidOutLimit);
}

}  // namespace

void setUp(void) {}
void tearDown(void) {}

// ---- stamp check: positive and negative cases ----

void test_real_stamps_ok(void) {
  // FAIL = gains header and vectors copied at different times, or contract bumped on one side.
  expect_result(Result::Ok, run_check(real_gains(), real_vecs()));
}

void test_stale_contract_gains(void) {
  Stamp g = real_gains();
  g.contract += 1;
  expect_result(Result::BadContract, run_check(g, real_vecs()));
}
void test_stale_contract_vectors(void) {
  Stamp v = real_vecs();
  v.contract += 1;
  expect_result(Result::BadContract, run_check(real_gains(), v));
}
void test_stale_schema_gains(void) {
  Stamp g = real_gains();
  g.schema += 1;
  expect_result(Result::BadSchema, run_check(g, real_vecs()));
}
void test_stale_schema_vectors(void) {
  Stamp v = real_vecs();
  v.schema += 1;
  expect_result(Result::BadSchema, run_check(real_gains(), v));
}
void test_stale_hash_vectors(void) {
  Stamp v = real_vecs();
  v.hash ^= 1u;  // single bit flip
  expect_result(Result::BadHash, run_check(real_gains(), v));
}
void test_stale_hash_gains(void) {
  Stamp g = real_gains();
  g.hash ^= 1u;
  expect_result(Result::BadHash, run_check(g, real_vecs()));
}
void test_expected_contract_mismatch(void) {
  // real stamps agree with each other but not with the expected contract
  expect_result(Result::BadContract,
                balance_stamp::check(real_gains(), real_vecs(), kExpectSchema, kExpectContract + 1));
}
void test_expected_schema_mismatch(void) {
  expect_result(Result::BadSchema,
                balance_stamp::check(real_gains(), real_vecs(), kExpectSchema + 1, kExpectContract));
}
void test_both_sides_stale_contract(void) {
  // gains and vectors agree with each other but both differ from the expectation
  Stamp g = real_gains(), v = real_vecs();
  g.contract += 1;
  v.contract += 1;
  expect_result(Result::BadContract, run_check(g, v));
}
void test_priority_schema_over_contract(void) {
  Stamp g = real_gains(), v = real_vecs();
  g.schema += 1;
  v.contract += 1;
  expect_result(Result::BadSchema, run_check(g, v));
}
void test_priority_contract_over_hash(void) {
  Stamp g = real_gains(), v = real_vecs();
  g.contract += 1;
  v.hash ^= 1u;
  expect_result(Result::BadContract, run_check(g, v));
}
void test_priority_schema_over_all(void) {
  Stamp g = real_gains(), v = real_vecs();
  g.schema += 1;
  v.contract += 1;
  v.hash ^= 1u;
  expect_result(Result::BadSchema, run_check(g, v));
}
void test_hash_both_flipped_documented_limit(void) {
  // Documents a known limit: hash is compared for EQUALITY only. Real drift detection is the
  // Python exporter --check, not this C++ check.
  Stamp g = real_gains(), v = real_vecs();
  g.hash ^= 1u;
  v.hash ^= 1u;
  expect_result(Result::Ok, run_check(g, v));
}

// ---- vectors shape and control ----

void test_vectors_nonempty(void) {
  TEST_ASSERT_TRUE_MESSAGE(kNLqr >= 20, "LQR vectors: need >= 20 cases");
  TEST_ASSERT_TRUE_MESSAGE(kNPid >= 50, "PID vectors: need >= 50 steps");
}

// Control: real gains give ZERO exceedances. Without this the discrimination tests prove nothing.
void test_control_real_gains_zero_exceedances(void) {
  const balance::LqrBalance lqr = lqr_with_k0_scale(1.0f);
  const Exceed l = lqr_exceed(lqr);
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, l.cmd, "LQR control: cmd exceeds kTolAbs with real gains");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, l.raw, "LQR control: raw exceeds kTolAbs with real gains");
  const Exceed p = pid_exceed(pid_with_kp_scale(1.0f));
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, p.cmd, "PID control: cmd exceeds kTolAbs with real gains");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, p.raw, "PID control: raw exceeds kTolAbs with real gains");
}

// ---- discrimination: a 1 percent gain error must be caught ----

// LQR: K0' = 1.01 * K0 = -24.0566142. raw shift = 0.01 * 23.8184299 * theta = 0.238184 * theta,
// theta = pitch + 0.1086. Hand numbers (kTolAbs = 6e-6):
//  idx 2  (pitch -0.1076): theta 0.001 -> shift 2.38e-4  (~40x tol), unsaturated, cmd catches it.
//  idx 8  (pitch -0.0986): theta 0.01  -> shift 2.38e-3  (~400x tol), unsaturated.
//  idx 1  (pitch 0):       theta 0.1086 -> raw shift 2.59e-2, but cmd stays clamped at 1.0,
//         so ONLY the raw comparison sees it. That is why raw is compared too.
// Rows with theta == 0 (idx 0,4,5,6,7,21) and fault rows (22,23) are blind; >= 16 rows are not.
void test_corrupted_gain_detected_lqr(void) {
  const Exceed e = lqr_exceed(lqr_with_k0_scale(1.01f));
  TEST_ASSERT_TRUE_MESSAGE(e.cmd >= 1, "LQR: no cmd exceedance with kLqrK[0]*1.01: vectors too weak");
  TEST_ASSERT_TRUE_MESSAGE(e.raw >= 1, "LQR: no raw exceedance with kLqrK[0]*1.01: vectors too weak");
}

// PID: kp' = 1.01 * kp. idx 1: error = 0.01 (pitch -0.01), dt 0.016.
// p = 0.01, i = 0.1*0.01*0.016 = 1.6e-5, d = 0.05*0.01/0.016 = 0.03125 -> raw 0.041266 (matches .inc).
// kp shift = 0.01 * 0.01 = 1.0e-4 (~16.7x tol), unsaturated, so cmd and raw both catch it.
void test_corrupted_gain_detected_pid(void) {
  const Exceed e = pid_exceed(pid_with_kp_scale(1.01f));
  TEST_ASSERT_TRUE_MESSAGE(e.cmd >= 1, "PID: no cmd exceedance with kPidKp*1.01: vectors too weak");
  TEST_ASSERT_TRUE_MESSAGE(e.raw >= 1, "PID: no raw exceedance with kPidKp*1.01: vectors too weak");
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_real_stamps_ok);
  RUN_TEST(test_stale_contract_gains);
  RUN_TEST(test_stale_contract_vectors);
  RUN_TEST(test_stale_schema_gains);
  RUN_TEST(test_stale_schema_vectors);
  RUN_TEST(test_stale_hash_vectors);
  RUN_TEST(test_stale_hash_gains);
  RUN_TEST(test_expected_contract_mismatch);
  RUN_TEST(test_expected_schema_mismatch);
  RUN_TEST(test_both_sides_stale_contract);
  RUN_TEST(test_priority_schema_over_contract);
  RUN_TEST(test_priority_contract_over_hash);
  RUN_TEST(test_priority_schema_over_all);
  RUN_TEST(test_hash_both_flipped_documented_limit);
  RUN_TEST(test_vectors_nonempty);
  RUN_TEST(test_control_real_gains_zero_exceedances);
  RUN_TEST(test_corrupted_gain_detected_lqr);
  RUN_TEST(test_corrupted_gain_detected_pid);
  return UNITY_END();
}
