#include <unity.h>
#include <balance_supervisor.h>
#include <cmath>
#include <cstring>
#include <limits>

using namespace balance_supervisor;

void setUp(void) {}
void tearDown(void) {}

static Config cfg() { return Config{-0.125f, 0.5f, 0.125f, 3, 3, 100000u}; }
static Tick okT(float pitch) { return Tick{20000u, false, true, pitch}; }
static Tick badT() { return Tick{20000u, false, false, 0.0f}; }

static Supervisor waiting() {
  Supervisor s(cfg());
  s.onStartup(true);
  s.onCalibration(true);
  return s;
}
// Running: three in-gate ticks (hold 3) -> Arm on the third.
static Supervisor running() {
  Supervisor s = waiting();
  (void)s.onTick(okT(-0.125f));
  (void)s.onTick(okT(-0.125f));
  TEST_ASSERT_EQUAL((int)Action::Arm, (int)s.onTick(okT(-0.125f)));
  return s;
}

void test_startup_and_calibration(void) {
  Supervisor s(cfg());
  TEST_ASSERT_EQUAL((int)State::Startup, (int)s.state());
  s.onStartup(true);
  TEST_ASSERT_EQUAL((int)State::Calibrating, (int)s.state());
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)s.onTick(okT(1.0f)));  // no motion while calibrating
  s.onCalibration(false);
  TEST_ASSERT_EQUAL((int)State::Calibrating, (int)s.state());
  s.onCalibration(true);
  TEST_ASSERT_EQUAL((int)State::WaitingForLean, (int)s.state());
}

void test_startup_failure_latches(void) {
  Supervisor s(cfg());
  s.onStartup(false);
  TEST_ASSERT_TRUE(s.latched());
  TEST_ASSERT_EQUAL((int)Cause::StartupFail, (int)s.cause());
}

void test_invalid_config_latches(void) {
  Supervisor a(Config{-0.125f, 0.1f, 0.5f, 3, 3, 100000u});  // gate > cutoff
  a.onStartup(true);
  TEST_ASSERT_EQUAL((int)Cause::StartupFail, (int)a.cause());
  Supervisor b(Config{std::numeric_limits<float>::quiet_NaN(), 0.5f, 0.125f, 3, 3, 100000u});
  b.onStartup(true);
  TEST_ASSERT_TRUE(b.latched());
}

// pitch 0.1: dev 0.225 > gate resets the hold. Sequence in,in,out,in,in,in arms on the 6th tick.
void test_gate_hold_and_reset(void) {
  Supervisor s = waiting();
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)s.onTick(okT(-0.125f)));
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)s.onTick(okT(-0.125f)));
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)s.onTick(okT(0.1f)));
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)s.onTick(okT(-0.125f)));
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)s.onTick(okT(-0.125f)));
  TEST_ASSERT_EQUAL((int)State::WaitingForLean, (int)s.state());
  TEST_ASSERT_EQUAL((int)Action::Arm, (int)s.onTick(okT(-0.125f)));
  TEST_ASSERT_EQUAL((int)State::Running, (int)s.state());
  TEST_ASSERT_EQUAL((int)Action::Drive, (int)s.onTick(okT(-0.125f)));
}

// pitch 0.0: dev 0.125 == gate -> inside (inclusive).
void test_gate_boundary_inclusive(void) {
  Supervisor s = waiting();
  (void)s.onTick(okT(0.0f));
  (void)s.onTick(okT(0.0f));
  TEST_ASSERT_EQUAL((int)Action::Arm, (int)s.onTick(okT(0.0f)));
}

void test_cutoff_boundary_and_trip(void) {
  Supervisor a = running();
  TEST_ASSERT_EQUAL((int)Action::Drive, (int)a.onTick(okT(0.375f)));  // dev 0.5 not > 0.5
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)a.onTick(okT(0.40625f))); // dev 0.53125
  TEST_ASSERT_EQUAL((int)Cause::TiltCutoff, (int)a.cause());
  Supervisor b = running();
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)b.onTick(okT(-0.65625f)));  // dev 0.53125
  TEST_ASSERT_EQUAL((int)Cause::TiltCutoff, (int)b.cause());
}

void test_imu_fail_consecutive(void) {
  Supervisor s = running();
  TEST_ASSERT_EQUAL((int)Action::Zero, (int)s.onTick(badT()));
  TEST_ASSERT_EQUAL((int)Action::Zero, (int)s.onTick(badT()));
  TEST_ASSERT_EQUAL((int)Action::Drive, (int)s.onTick(okT(-0.125f)));  // resets the count
  TEST_ASSERT_EQUAL((int)Action::Zero, (int)s.onTick(badT()));
  TEST_ASSERT_EQUAL((int)Action::Zero, (int)s.onTick(badT()));
  TEST_ASSERT_EQUAL((int)State::Running, (int)s.state());
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)s.onTick(badT()));  // third in a row
  TEST_ASSERT_EQUAL((int)Cause::ImuFail, (int)s.cause());
}

// ok,ok,bad,ok,ok -> still waiting (bad reset the hold); the next ok arms.
void test_imu_miss_resets_gate(void) {
  Supervisor s = waiting();
  (void)s.onTick(okT(-0.125f));
  (void)s.onTick(okT(-0.125f));
  (void)s.onTick(badT());
  (void)s.onTick(okT(-0.125f));
  (void)s.onTick(okT(-0.125f));
  TEST_ASSERT_EQUAL((int)State::WaitingForLean, (int)s.state());
  TEST_ASSERT_EQUAL((int)Action::Arm, (int)s.onTick(okT(-0.125f)));
}

void test_nonfinite(void) {
  Supervisor a = running();
  (void)a.onTick(okT(std::numeric_limits<float>::quiet_NaN()));
  TEST_ASSERT_EQUAL((int)Cause::NonFinite, (int)a.cause());
  Supervisor b = running();
  (void)b.onTick(okT(std::numeric_limits<float>::infinity()));
  TEST_ASSERT_EQUAL((int)Cause::NonFinite, (int)b.cause());
  Supervisor c = running();
  TEST_ASSERT_FALSE(c.onCommand(std::numeric_limits<float>::quiet_NaN(), false));
  TEST_ASSERT_EQUAL((int)Cause::NonFinite, (int)c.cause());
  Supervisor d = running();
  TEST_ASSERT_FALSE(d.onCommand(0.5f, true));  // ctrl fault flag
  TEST_ASSERT_TRUE(d.latched());
  Supervisor e = running();
  TEST_ASSERT_TRUE(e.onCommand(0.5f, false));
  TEST_ASSERT_EQUAL((int)State::Running, (int)e.state());
  Supervisor w = waiting();
  TEST_ASSERT_FALSE(w.onCommand(0.5f, false));  // not running: refuses, does not latch
  TEST_ASSERT_FALSE(w.latched());
}

void test_stall(void) {
  Supervisor a = running();
  TEST_ASSERT_EQUAL((int)Action::Drive, (int)a.onTick(Tick{100000u, false, true, -0.125f}));  // == limit ok
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)a.onTick(Tick{100001u, false, true, -0.125f}));
  TEST_ASSERT_EQUAL((int)Cause::Stall, (int)a.cause());
  Supervisor b = running();  // first tick: gap is not a measurement
  TEST_ASSERT_EQUAL((int)Action::Drive, (int)b.onTick(Tick{5000000u, true, true, -0.125f}));
  Supervisor w = waiting();  // not Running: no stall check
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)w.onTick(Tick{5000000u, false, true, -0.125f}));
  TEST_ASSERT_FALSE(w.latched());
}

void test_precedence_stall_first(void) {
  Supervisor s = running();
  (void)s.onTick(Tick{100001u, false, true, 0.5f});  // stall AND beyond cutoff
  TEST_ASSERT_EQUAL((int)Cause::Stall, (int)s.cause());
  Supervisor t = running();
  (void)t.onTick(Tick{100001u, false, false, 0.0f});  // stall AND imu miss
  TEST_ASSERT_EQUAL((int)Cause::Stall, (int)t.cause());
}

void test_wheels_dead(void) {
  Supervisor r = running();
  r.onWheels(true);
  TEST_ASSERT_EQUAL((int)State::Running, (int)r.state());
  r.onWheels(false);
  TEST_ASSERT_EQUAL((int)Cause::WheelFail, (int)r.cause());
  Supervisor c(cfg());
  c.onStartup(true);
  c.onWheels(false);
  TEST_ASSERT_EQUAL((int)Cause::WheelFail, (int)c.cause());
  Supervisor s(cfg());
  s.onWheels(false);  // Startup: ignored (onStartup owns that)
  TEST_ASSERT_EQUAL((int)State::Startup, (int)s.state());
}

void test_serial_kill_any_state(void) {
  Supervisor a = running();
  a.onKillRequest(Cause::SerialKill);
  TEST_ASSERT_EQUAL((int)Cause::SerialKill, (int)a.cause());
  Supervisor b = waiting();
  b.onKillRequest(Cause::SerialKill);
  TEST_ASSERT_TRUE(b.latched());
  Supervisor c(cfg());
  c.onStartup(true);
  c.onKillRequest(Cause::SerialKill);
  TEST_ASSERT_TRUE(c.latched());
}

void test_latched_is_terminal_first_cause_wins(void) {
  Supervisor s = running();
  (void)s.onTick(okT(0.40625f));  // TiltCutoff
  s.onKillRequest(Cause::SerialKill);
  TEST_ASSERT_EQUAL((int)Cause::TiltCutoff, (int)s.cause());
  s.onStartup(true);
  s.onCalibration(true);
  TEST_ASSERT_EQUAL((int)Action::Idle, (int)s.onTick(okT(-0.125f)));
  TEST_ASSERT_FALSE(s.onCommand(0.1f, false));
  TEST_ASSERT_EQUAL((int)State::Latched, (int)s.state());
}

void test_names(void) {
  TEST_ASSERT_EQUAL_STRING("Running", name(State::Running));
  TEST_ASSERT_EQUAL_STRING("tilt", name(Cause::TiltCutoff));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_startup_and_calibration);
  RUN_TEST(test_startup_failure_latches);
  RUN_TEST(test_invalid_config_latches);
  RUN_TEST(test_gate_hold_and_reset);
  RUN_TEST(test_gate_boundary_inclusive);
  RUN_TEST(test_cutoff_boundary_and_trip);
  RUN_TEST(test_imu_fail_consecutive);
  RUN_TEST(test_imu_miss_resets_gate);
  RUN_TEST(test_nonfinite);
  RUN_TEST(test_stall);
  RUN_TEST(test_precedence_stall_first);
  RUN_TEST(test_wheels_dead);
  RUN_TEST(test_serial_kill_any_state);
  RUN_TEST(test_latched_is_terminal_first_cause_wins);
  RUN_TEST(test_names);
  return UNITY_END();
}
