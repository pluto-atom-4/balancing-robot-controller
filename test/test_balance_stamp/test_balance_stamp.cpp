#include <unity.h>
#include <cmath>
#include <cstdint>
#include <hal_iface.h>
#include <balance_core.h>
#include <balance_gains_generated.h>

// Internal-consistency check of the committed GENERATED gains header.
// Cross-repo drift is NOT checked here: run, from freecad-workspace/inverted-pendulum-project,
//   mamba run -n pendulum-tools python3 07_Simulation/hal/export_cpp.py --cpp-repo <this repo> --check
// Values are SIM-derived (linearized plant), NOT tuned or validated on the real robot.
// Nothing here runs on hardware. kGainsHash is only pinned (C++ cannot recompute it):
// when gains are regenerated from Python, update the pinned numbers below on purpose.
// kGainsHash == kVecGainsHash is checked by #29 / #42, not here.

static_assert(balance_gains::kContractVersion == hal::kContractVersion,
              "generated gains contract != hal_iface.h contract (re-export or bump both)");
static_assert(balance_gains::kSchema == 1, "unexpected gains schema");

namespace {
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
constexpr float kTol = 1e-4f;
}  // namespace

void setUp(void) {}
void tearDown(void) {}

void test_stamp_schema_and_contract(void) {
  TEST_ASSERT_EQUAL_UINT32(1u, balance_gains::kSchema);
  TEST_ASSERT_EQUAL_UINT32(hal::kContractVersion, balance_gains::kContractVersion);
}

void test_stamp_hash_pinned_nonzero(void) {
  TEST_ASSERT_TRUE(balance_gains::kGainsHash != 0u);
  TEST_ASSERT_EQUAL_HEX32(0x1e60b0fcu, balance_gains::kGainsHash);
}

void test_lqr_k_finite_nonzero_signs(void) {
  for (int i = 0; i < 2; ++i) {
    TEST_ASSERT_TRUE(std::isfinite(balance_gains::kLqrK[i]));
    TEST_ASSERT_TRUE(balance_gains::kLqrK[i] != 0.0f);
    TEST_ASSERT_TRUE(balance_gains::kLqrK[i] < 0.0f);  // both negative in this export
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -23.8184299f, balance_gains::kLqrK[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -3.49402261f, balance_gains::kLqrK[1]);
}

void test_theta_ref_is_sim_value(void) {
  // exact float equality: -0.108599998f is the 9-digit print of float -0.1086f
  TEST_ASSERT_TRUE(balance_gains::kThetaRefRad == -0.1086f);
}

void test_limits_positive_and_one(void) {
  TEST_ASSERT_TRUE(balance_gains::kOutLimit > 0.0f);
  TEST_ASSERT_TRUE(balance_gains::kPidOutLimit > 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, balance_gains::kOutLimit);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, balance_gains::kPidOutLimit);
}

void test_pid_gains_values(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, balance_gains::kPidKp);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, balance_gains::kPidKi);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, balance_gains::kPidKd);
}

// pitch == theta_ref, gyro 0: theta = 0 -> raw 0, cmd 0 (hand: -(K0*0 + K1*0))
void test_lqr_at_theta_ref_is_zero(void) {
  balance::ControlOutput o = make_lqr().step(make_imu(-0.1086f, 0.0f), 0.02f);
  TEST_ASSERT_FALSE(o.fault);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.cmd_rad_s);
}

// pitch 0, gyro 0: theta = 0.1086; raw = 23.8184299*0.1086 = 2.58668 -> clamped to 1.0
void test_lqr_pitch_zero_saturates(void) {
  balance::ControlOutput o = make_lqr().step(make_imu(0.0f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 2.58668f, o.raw_cmd);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, o.cmd_rad_s);
}

// pitch = ref + 0.01, gyro 0: raw = 0.238184 (unsaturated); mirror -0.01 -> -0.238184
void test_lqr_small_offset_both_signs(void) {
  balance::LqrBalance lqr = make_lqr();
  balance::ControlOutput a = lqr.step(make_imu(-0.0986f, 0.0f), 0.02f);
  balance::ControlOutput b = lqr.step(make_imu(-0.1186f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.238184f, a.cmd_rad_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.238184f, b.cmd_rad_s);
}

// pitch = ref + 0.01, gyro 0.1: raw = 0.238184 + 0.349402 = 0.587586
void test_lqr_gyro_term(void) {
  balance::ControlOutput o = make_lqr().step(make_imu(-0.0986f, 0.1f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.587586f, o.cmd_rad_s);
}

// pid first step: pitch -0.1, dt 0.02: p=0.1, i=0.1*0.002=0.0002, d=0 -> 0.1002
void test_pid_first_step_from_gains(void) {
  balance::ControlOutput o = make_pid().step(make_imu(-0.1f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.1002f, o.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.1002f, o.cmd_rad_s);
  TEST_ASSERT_FALSE(o.fault);
}

// pid saturates: pitch -2 -> p=2 -> cmd clamped to 1.0
void test_pid_saturates(void) {
  balance::ControlOutput o = make_pid().step(make_imu(-2.0f, 0.0f), 0.02f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, o.cmd_rad_s);
  TEST_ASSERT_TRUE(o.raw_cmd > 1.0f);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_stamp_schema_and_contract);
  RUN_TEST(test_stamp_hash_pinned_nonzero);
  RUN_TEST(test_lqr_k_finite_nonzero_signs);
  RUN_TEST(test_theta_ref_is_sim_value);
  RUN_TEST(test_limits_positive_and_one);
  RUN_TEST(test_pid_gains_values);
  RUN_TEST(test_lqr_at_theta_ref_is_zero);
  RUN_TEST(test_lqr_pitch_zero_saturates);
  RUN_TEST(test_lqr_small_offset_both_signs);
  RUN_TEST(test_lqr_gyro_term);
  RUN_TEST(test_pid_first_step_from_gains);
  RUN_TEST(test_pid_saturates);
  return UNITY_END();
}
