#include <unity.h>
#include <hal_iface.h>
#include <balance_core.h>
#include <limits>

// Hand-computed from the formulas in balance_core.h. NOT a parity test
// (parity vs Python is #29). Native only; nothing here runs on hardware.
// -0.1086 is the SIM theta_ref, used only as a test input.

namespace {
hal::ImuSample make_imu(float pitch, float gyro_y) {
  hal::ImuSample s{};
  s.pitch_rad = pitch;
  s.gyro_rad_s[1] = gyro_y;
  return s;
}
constexpr float kTol = 1e-5f;
}  // namespace

void setUp(void) {}
void tearDown(void) {}

// ---------------- PID: kp=1, ki=0.1, kd=0.05, limit=1 ----------------

// pitch 0: error 0, p=0, i=0, d=0 -> 0
void test_pid_zero_input(void) {
  balance::PidBalance pid(1.0f, 0.1f, 0.05f, 1.0f);
  balance::ControlOutput o = pid.step(make_imu(0.0f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.cmd_rad_s);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.raw_cmd);
  TEST_ASSERT_FALSE(o.fault);
}

// pitch -0.1, dt 0.02: error=0.1; p=0.1; integral=0.1*0.02=0.002; i=0.1*0.002=0.0002;
// first call d=0; raw=0.1002
void test_pid_first_step(void) {
  balance::PidBalance pid(1.0f, 0.1f, 0.05f, 1.0f);
  balance::ControlOutput o = pid.step(make_imu(-0.1f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.1002f, o.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.1002f, o.cmd_rad_s);
  TEST_ASSERT_FALSE(o.fault);
}

// after first step, pitch -0.2: error=0.2; p=0.2; integral=0.002+0.2*0.02=0.006; i=0.0006;
// d=0.05*(0.2-0.1)/0.02=0.25; raw=0.2+0.0006+0.25=0.4506
void test_pid_second_step_derivative(void) {
  balance::PidBalance pid(1.0f, 0.1f, 0.05f, 1.0f);
  pid.step(make_imu(-0.1f, 0.0f), 0.02f);
  balance::ControlOutput o = pid.step(make_imu(-0.2f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.4506f, o.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.4506f, o.cmd_rad_s);
}

// third step pitch -1.5: error=1.5; p=1.5; integral=0.006+1.5*0.02=0.036; i=0.0036;
// d=0.05*(1.5-0.2)/0.02=3.25; raw=1.5+0.0036+3.25=4.7536; cmd clamped to 1.0
void test_pid_saturation_positive(void) {
  balance::PidBalance pid(1.0f, 0.1f, 0.05f, 1.0f);
  pid.step(make_imu(-0.1f, 0.0f), 0.02f);
  pid.step(make_imu(-0.2f, 0.0f), 0.02f);
  balance::ControlOutput o = pid.step(make_imu(-1.5f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 4.7536f, o.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f, o.cmd_rad_s);
  TEST_ASSERT_FALSE(o.fault);
}

// fresh, pitch +5, dt 1.0: error=-5; integral=-5 clamped to -1 (limit); i=0.1*-1=-0.1;
// d=0 (first call); p=-5; raw=-5.1; cmd=-1.0
void test_pid_saturation_negative_and_integral_clamp(void) {
  balance::PidBalance pid(1.0f, 0.1f, 0.05f, 1.0f);
  balance::ControlOutput o = pid.step(make_imu(5.0f, 0.0f), 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -5.1f, o.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, -1.0f, o.cmd_rad_s);
}

// dt==0: P-only. pitch -0.3: error=0.3; raw=kp*0.3=0.3; cmd 0.3.
// then pitch -2.0: raw=2.0; cmd clamped 1.0. No state update, so a following
// dt>0 first step behaves like a fresh controller: pitch -0.1, dt 0.02 -> 0.1002.
void test_pid_dt_zero_p_only_no_state(void) {
  balance::PidBalance pid(1.0f, 0.1f, 0.05f, 1.0f);
  balance::ControlOutput a = pid.step(make_imu(-0.3f, 0.0f), 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.3f, a.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.3f, a.cmd_rad_s);
  TEST_ASSERT_FALSE(a.fault);
  balance::ControlOutput b = pid.step(make_imu(-2.0f, 0.0f), 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 2.0f, b.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f, b.cmd_rad_s);
  balance::ControlOutput c = pid.step(make_imu(-0.1f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.1002f, c.raw_cmd);
}

// mixed: step(-0.1,0.02); step(-0.2,0.0) [no state change]; step(-0.2,0.02):
// error=0.2, prev_error=0.1 (from step 1), integral=0.006, d=0.25 -> raw=0.4506
void test_pid_dt_zero_keeps_prev_error(void) {
  balance::PidBalance pid(1.0f, 0.1f, 0.05f, 1.0f);
  pid.step(make_imu(-0.1f, 0.0f), 0.02f);
  pid.step(make_imu(-0.2f, 0.0f), 0.0f);
  balance::ControlOutput o = pid.step(make_imu(-0.2f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.4506f, o.cmd_rad_s);
}

// dt<0: {0,0,true}, state untouched; next normal first step = 0.1002
void test_pid_dt_negative_fault(void) {
  balance::PidBalance pid(1.0f, 0.1f, 0.05f, 1.0f);
  balance::ControlOutput o = pid.step(make_imu(-0.1f, 0.0f), -0.01f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.cmd_rad_s);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.raw_cmd);
  TEST_ASSERT_TRUE(o.fault);
  balance::ControlOutput n = pid.step(make_imu(-0.1f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.1002f, n.cmd_rad_s);
  TEST_ASSERT_FALSE(n.fault);
}

// NaN dt is treated as a fault (conservative; deviates from Python only for NaN)
void test_pid_dt_nan_fault(void) {
  balance::PidBalance pid(1.0f, 0.1f, 0.05f, 1.0f);
  balance::ControlOutput o =
      pid.step(make_imu(-0.1f, 0.0f), std::numeric_limits<float>::quiet_NaN());
  TEST_ASSERT_TRUE(o.fault);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.cmd_rad_s);
}

// reset clears integral and prev_error: after steps 1..3 and reset(),
// step(-0.1,0.02) is again 0.1002 (integral 0.0002, d=0)
void test_pid_reset_clears_state(void) {
  balance::PidBalance pid(1.0f, 0.1f, 0.05f, 1.0f);
  pid.step(make_imu(-0.1f, 0.0f), 0.02f);
  pid.step(make_imu(-0.2f, 0.0f), 0.02f);
  pid.step(make_imu(-1.5f, 0.0f), 0.02f);
  pid.reset();
  balance::ControlOutput o = pid.step(make_imu(-0.1f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.1002f, o.raw_cmd);
}

// non-positive limit -> output always 0 (fail-safe, C++ has no ValueError)
void test_pid_nonpositive_limit_outputs_zero(void) {
  balance::PidBalance pid(1.0f, 0.1f, 0.05f, 0.0f);
  balance::ControlOutput o = pid.step(make_imu(-0.3f, 0.0f), 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.cmd_rad_s);
}

// ---------------- LQR: K=(10,2), theta_ref=-0.1086 (sim value), limit=1 ----------------
// raw = -(10*theta + 2*gyro_y), theta = pitch - theta_ref

// pitch == ref, gyro 0: theta=0 exactly; raw=-(0+0)=0
void test_lqr_at_reference_is_zero(void) {
  balance::LqrBalance lqr(10.0f, 2.0f, -0.1086f, 1.0f);
  balance::ControlOutput o = lqr.step(make_imu(-0.1086f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.cmd_rad_s);
  TEST_ASSERT_FALSE(o.fault);
}

// pitch -0.0586: theta=-0.0586+0.1086=0.05; raw=-(10*0.05)=-0.5 (sign: positive theta -> negative cmd)
void test_lqr_sign_positive_theta(void) {
  balance::LqrBalance lqr(10.0f, 2.0f, -0.1086f, 1.0f);
  balance::ControlOutput o = lqr.step(make_imu(-0.0586f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, -0.5f, o.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, -0.5f, o.cmd_rad_s);
}

// pitch = ref (theta=0), gyro_y=+0.3: raw=-(0+2*0.3)=-0.6; gyro_y=-0.3: raw=+0.6
void test_lqr_gyro_term_sign(void) {
  balance::LqrBalance lqr(10.0f, 2.0f, -0.1086f, 1.0f);
  balance::ControlOutput a = lqr.step(make_imu(-0.1086f, 0.3f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, -0.6f, a.raw_cmd);
  balance::ControlOutput b = lqr.step(make_imu(-0.1086f, -0.3f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.6f, b.raw_cmd);
}

// theta=0.05, gyro 0.1: raw=-(0.5+0.2)=-0.7
void test_lqr_gyro_added(void) {
  balance::LqrBalance lqr(10.0f, 2.0f, -0.1086f, 1.0f);
  balance::ControlOutput o = lqr.step(make_imu(-0.0586f, 0.1f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, -0.7f, o.raw_cmd);
}

// pitch 0.0914, gyro 0.5: theta=0.2; raw=-(2.0+1.0)=-3.0; cmd=-1.0
// pitch -0.2086, gyro -1.5: theta=-0.1; raw=-(-1.0-3.0)=4.0; cmd=+1.0
void test_lqr_saturation_both_sides(void) {
  balance::LqrBalance lqr(10.0f, 2.0f, -0.1086f, 1.0f);
  balance::ControlOutput a = lqr.step(make_imu(0.0914f, 0.5f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -3.0f, a.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, -1.0f, a.cmd_rad_s);
  balance::ControlOutput b = lqr.step(make_imu(-0.2086f, -1.5f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.0f, b.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f, b.cmd_rad_s);
}

// out_limit 0.5, raw=-0.7 -> cmd=-0.5
void test_lqr_custom_limit(void) {
  balance::LqrBalance lqr(10.0f, 2.0f, -0.1086f, 0.5f);
  balance::ControlOutput o = lqr.step(make_imu(-0.0586f, 0.1f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.7f, o.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(kTol, -0.5f, o.cmd_rad_s);
}

// theta_ref=0, pitch 0.05, gyro 0: theta=0.05; raw=-0.5
void test_lqr_custom_theta_ref(void) {
  balance::LqrBalance lqr(10.0f, 2.0f, 0.0f, 1.0f);
  balance::ControlOutput o = lqr.step(make_imu(0.05f, 0.0f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(kTol, -0.5f, o.raw_cmd);
}

// dt in {0, 0.016, 0.032} gives same output -0.7, fault false
void test_lqr_dt_ignored(void) {
  balance::LqrBalance lqr(10.0f, 2.0f, -0.1086f, 1.0f);
  const float dts[3] = {0.0f, 0.016f, 0.032f};
  for (int i = 0; i < 3; ++i) {
    balance::ControlOutput o = lqr.step(make_imu(-0.0586f, 0.1f), dts[i]);
    TEST_ASSERT_FLOAT_WITHIN(kTol, -0.7f, o.cmd_rad_s);
    TEST_ASSERT_FALSE(o.fault);
  }
}

// dt<0 -> {0,0,true}
void test_lqr_dt_negative_fault(void) {
  balance::LqrBalance lqr(10.0f, 2.0f, -0.1086f, 1.0f);
  balance::ControlOutput o = lqr.step(make_imu(-0.0586f, 0.1f), -0.001f);
  TEST_ASSERT_TRUE(o.fault);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.cmd_rad_s);
  TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, o.raw_cmd);
}

// negative K (shape of the sim gains, NOT hardware-validated):
// K=(-23.81842951,-3.49402271), pitch -0.0986, gyro 0.1: theta=0.01;
// raw=-(-23.81842951*0.01 + -3.49402271*0.1) = 0.2381843+0.3494023 = 0.5875866 (below limit)
void test_lqr_negative_k_shape(void) {
  balance::LqrBalance lqr(-23.81842951f, -3.49402271f, -0.1086f, 1.0f);
  balance::ControlOutput o = lqr.step(make_imu(-0.0986f, 0.1f), 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5875866f, o.raw_cmd);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5875866f, o.cmd_rad_s);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_pid_zero_input);
  RUN_TEST(test_pid_first_step);
  RUN_TEST(test_pid_second_step_derivative);
  RUN_TEST(test_pid_saturation_positive);
  RUN_TEST(test_pid_saturation_negative_and_integral_clamp);
  RUN_TEST(test_pid_dt_zero_p_only_no_state);
  RUN_TEST(test_pid_dt_zero_keeps_prev_error);
  RUN_TEST(test_pid_dt_negative_fault);
  RUN_TEST(test_pid_dt_nan_fault);
  RUN_TEST(test_pid_reset_clears_state);
  RUN_TEST(test_pid_nonpositive_limit_outputs_zero);
  RUN_TEST(test_lqr_at_reference_is_zero);
  RUN_TEST(test_lqr_sign_positive_theta);
  RUN_TEST(test_lqr_gyro_term_sign);
  RUN_TEST(test_lqr_gyro_added);
  RUN_TEST(test_lqr_saturation_both_sides);
  RUN_TEST(test_lqr_custom_limit);
  RUN_TEST(test_lqr_custom_theta_ref);
  RUN_TEST(test_lqr_dt_ignored);
  RUN_TEST(test_lqr_dt_negative_fault);
  RUN_TEST(test_lqr_negative_k_shape);
  return UNITY_END();
}
