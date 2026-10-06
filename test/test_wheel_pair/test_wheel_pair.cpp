#include <unity.h>
#include <wheel_servo.h>
#include <cstdint>
#include <limits>

using wheel_servo::DeviceInfo;
using wheel_servo::Fault;
using wheel_servo::IWheelServo;
using wheel_servo::WheelPair;
using wheel_servo::WheelPairConfig;

// FakeServo scripts failures per id (ids used: 1 and 2). Records calls and call order.
// Units kXl330: 1 raw = 0.229 * 0.104719755 = 0.0239808 rad/s.
// 0.5 rad/s -> 20.85 -> 21 ; 1.0 -> 41.70 -> 42 ; limit 30 raw -> 0.7194 rad/s cap ;
// default limit 1620 -> 38.85 rad/s.
class FakeServo : public IWheelServo {
 public:
  bool begin_ok = true;
  bool ping_ok[4] = {true, true, true, true};
  bool torque_off_ok[4] = {true, true, true, true};
  bool info_ok = true;
  bool have_limit = true;
  int32_t limit_raw = 1620;
  bool mode_ok = true;
  bool vel_ok[4] = {true, true, true, true};

  int begin_calls = 0;
  int torque_off_calls[4] = {0, 0, 0, 0};
  int mode_calls = 0;
  int vel_calls = 0;
  int32_t last_raw[4] = {0, 0, 0, 0};
  int tick = 0;
  int first_off_tick = 0;
  int first_mode_tick = 0;
  int first_vel_tick = 0;

  bool begin(uint32_t) override { ++begin_calls; return begin_ok; }
  bool ping(uint8_t id) override { return ping_ok[id]; }
  bool readInfo(uint8_t, DeviceInfo& out) override {
    if (!info_ok) return false;
    out.model_number = 1;
    out.have_vel_limit = have_limit;
    out.vel_limit_raw = limit_raw;
    return true;
  }
  bool torqueOff(uint8_t id) override {
    ++torque_off_calls[id];
    if (first_off_tick == 0) first_off_tick = ++tick;
    return torque_off_ok[id];
  }
  bool enableVelocityMode(uint8_t) override {
    ++mode_calls;
    if (first_mode_tick == 0) first_mode_tick = ++tick;
    return mode_ok;
  }
  bool setVelocityRaw(uint8_t id, int32_t raw) override {
    ++vel_calls;
    if (first_vel_tick == 0) first_vel_tick = ++tick;
    last_raw[id] = raw;
    return vel_ok[id];
  }
  bool presentVelocityRaw(uint8_t id, int32_t& out) override { out = last_raw[id]; return true; }
  const dxl_units::VelocityUnits& units() const override { return dxl_units::kXl330; }
  const char* family() const override { return "FAKE"; }
  Fault lastFault() const override { return Fault::None; }
};

static FakeServo fake;

void setUp(void) { fake = FakeServo(); }
void tearDown(void) {}

static WheelPairConfig mkCfg(float max_rad_s = 1.0f, uint8_t fail_max = 3) {
  WheelPairConfig c = {1, 2, 1, -1, max_rad_s, fail_max};  // left +1, right mirrored -1
  return c;
}
static int F(Fault f) { return static_cast<int>(f); }

// begin: torque off BEFORE mode change BEFORE any velocity write; ends with goal 0 both.
void test_begin_ok_and_order(void) {
  WheelPair wp(fake, mkCfg());
  TEST_ASSERT_TRUE(wp.begin(1u));
  TEST_ASSERT_TRUE(wp.alive());
  TEST_ASSERT_EQUAL_INT(F(Fault::None), F(wp.fault()));
  TEST_ASSERT_EQUAL_INT(1, fake.begin_calls);
  TEST_ASSERT_TRUE(fake.first_off_tick > 0);
  TEST_ASSERT_TRUE(fake.first_off_tick < fake.first_mode_tick);
  TEST_ASSERT_TRUE(fake.first_mode_tick < fake.first_vel_tick);
  TEST_ASSERT_EQUAL_INT(2, fake.vel_calls);  // goal 0 for both ids
  TEST_ASSERT_EQUAL_INT32(0, fake.last_raw[1]);
  TEST_ASSERT_EQUAL_INT32(0, fake.last_raw[2]);
}

void test_zero_command(void) {
  WheelPair wp(fake, mkCfg());
  TEST_ASSERT_TRUE(wp.begin(1u));
  TEST_ASSERT_TRUE(wp.writeRadS(0.0f, 0.0f));
  TEST_ASSERT_EQUAL_INT32(0, fake.last_raw[1]);
  TEST_ASSERT_EQUAL_INT32(0, fake.last_raw[2]);
}

// 0.5 rad/s -> 21 raw. right sign -1: (0.5,0.5) -> L +21, R -21 ; (0.5,-0.5) -> L +21, R +21.
void test_sign_per_wheel(void) {
  WheelPair wp(fake, mkCfg());
  TEST_ASSERT_TRUE(wp.begin(1u));
  TEST_ASSERT_TRUE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_EQUAL_INT32(21, fake.last_raw[1]);
  TEST_ASSERT_EQUAL_INT32(-21, fake.last_raw[2]);
  TEST_ASSERT_TRUE(wp.writeRadS(0.5f, -0.5f));
  TEST_ASSERT_EQUAL_INT32(21, fake.last_raw[1]);
  TEST_ASSERT_EQUAL_INT32(21, fake.last_raw[2]);
}

// max_rad_s 0.5: (2.0,-2.0) clamps to (0.5,-0.5) -> L +21, R (-21 * -1) = +21.
void test_clamp_max_rad_s(void) {
  WheelPair wp(fake, mkCfg(0.5f));
  TEST_ASSERT_TRUE(wp.begin(1u));
  TEST_ASSERT_TRUE(wp.writeRadS(2.0f, -2.0f));
  TEST_ASSERT_EQUAL_INT32(21, fake.last_raw[1]);
  TEST_ASSERT_EQUAL_INT32(21, fake.last_raw[2]);
}

// device limit 30 raw = 30*0.0239808 = 0.7194 rad/s < max_rad_s 1.0 -> cmd 1.0 clamps to 30 raw. R sign -1 -> -30.
void test_clamp_device_limit(void) {
  fake.limit_raw = 30;
  WheelPair wp(fake, mkCfg(1.0f));
  TEST_ASSERT_TRUE(wp.begin(1u));
  TEST_ASSERT_TRUE(wp.writeRadS(1.0f, 1.0f));
  TEST_ASSERT_EQUAL_INT32(30, fake.last_raw[1]);
  TEST_ASSERT_EQUAL_INT32(-30, fake.last_raw[2]);
}

// no device limit read -> fallback default 1620 (38.85 rad/s); max_rad_s 1.0 dominates: 1.0 -> 42 raw.
void test_limit_fallback_default(void) {
  fake.have_limit = false;
  WheelPair wp(fake, mkCfg(1.0f));
  TEST_ASSERT_TRUE(wp.begin(1u));
  TEST_ASSERT_TRUE(wp.writeRadS(1.0f, 1.0f));
  TEST_ASSERT_EQUAL_INT32(42, fake.last_raw[1]);
  TEST_ASSERT_EQUAL_INT32(-42, fake.last_raw[2]);
}

// NaN / inf -> 0, no latch.
void test_nan_inf_to_zero(void) {
  WheelPair wp(fake, mkCfg());
  TEST_ASSERT_TRUE(wp.begin(1u));
  TEST_ASSERT_TRUE(wp.writeRadS(0.5f, 0.5f));
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  TEST_ASSERT_TRUE(wp.writeRadS(nan, inf));
  TEST_ASSERT_EQUAL_INT32(0, fake.last_raw[1]);
  TEST_ASSERT_EQUAL_INT32(0, fake.last_raw[2]);
  TEST_ASSERT_TRUE(wp.alive());
}

// begin: torque off calls = 1 each. fail_max 3: after 3 failed writes -> latch + torqueOff on both (2 each),
// later writes do nothing. Each failed write makes 2 setVelocityRaw calls (default pair tries both).
void test_latch_after_fail_max(void) {
  WheelPair wp(fake, mkCfg(1.0f, 3));
  TEST_ASSERT_TRUE(wp.begin(1u));
  TEST_ASSERT_EQUAL_INT(1, fake.torque_off_calls[1]);
  TEST_ASSERT_EQUAL_INT(1, fake.torque_off_calls[2]);
  const int base = fake.vel_calls;  // 2
  fake.vel_ok[1] = false;
  fake.vel_ok[2] = false;
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_TRUE(wp.alive());
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_TRUE(wp.alive());
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_FALSE(wp.alive());
  TEST_ASSERT_EQUAL_INT(F(Fault::WriteFail), F(wp.fault()));
  TEST_ASSERT_EQUAL_INT(2, fake.torque_off_calls[1]);
  TEST_ASSERT_EQUAL_INT(2, fake.torque_off_calls[2]);
  TEST_ASSERT_EQUAL_INT(base + 6, fake.vel_calls);
  fake.vel_ok[1] = true;
  fake.vel_ok[2] = true;
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));  // latched: suppressed even if the bus recovers
  TEST_ASSERT_EQUAL_INT(base + 6, fake.vel_calls);
}

// failures must be CONSECUTIVE: fail,fail,ok,fail,fail still alive with fail_max 3; one more fail latches.
void test_fail_count_resets_on_success(void) {
  WheelPair wp(fake, mkCfg(1.0f, 3));
  TEST_ASSERT_TRUE(wp.begin(1u));
  fake.vel_ok[1] = false;
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));
  fake.vel_ok[1] = true;
  TEST_ASSERT_TRUE(wp.writeRadS(0.5f, 0.5f));
  fake.vel_ok[1] = false;
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_TRUE(wp.alive());
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_FALSE(wp.alive());
}

// unconfirmed torque-off at begin -> not alive, fault TorqueOffUnconfirmed, torque off still tried on id 2,
// no mode change, no velocity write.
void test_unconfirmed_torque_off(void) {
  fake.torque_off_ok[1] = false;
  WheelPair wp(fake, mkCfg());
  TEST_ASSERT_FALSE(wp.begin(1u));
  TEST_ASSERT_FALSE(wp.alive());
  TEST_ASSERT_EQUAL_INT(F(Fault::TorqueOffUnconfirmed), F(wp.fault()));
  TEST_ASSERT_TRUE(fake.torque_off_calls[2] >= 1);
  TEST_ASSERT_EQUAL_INT(0, fake.mode_calls);
  TEST_ASSERT_EQUAL_INT(0, fake.vel_calls);
}

// ping failing on id 2 -> begin false, torque off tried on both ids, no mode change, no velocity write.
void test_ping_fail_no_velocity_write(void) {
  fake.ping_ok[2] = false;
  WheelPair wp(fake, mkCfg());
  TEST_ASSERT_FALSE(wp.begin(1u));
  TEST_ASSERT_FALSE(wp.alive());
  TEST_ASSERT_EQUAL_INT(F(Fault::NoPing), F(wp.fault()));
  TEST_ASSERT_TRUE(fake.torque_off_calls[1] >= 1);
  TEST_ASSERT_TRUE(fake.torque_off_calls[2] >= 1);
  TEST_ASSERT_EQUAL_INT(0, fake.mode_calls);
  TEST_ASSERT_EQUAL_INT(0, fake.vel_calls);
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_EQUAL_INT(0, fake.vel_calls);
}

void test_readinfo_fail(void) {
  fake.info_ok = false;
  WheelPair wp(fake, mkCfg());
  TEST_ASSERT_FALSE(wp.begin(1u));
  TEST_ASSERT_EQUAL_INT(F(Fault::ReadFail), F(wp.fault()));
  TEST_ASSERT_EQUAL_INT(0, fake.mode_calls);
  TEST_ASSERT_EQUAL_INT(0, fake.vel_calls);
}

// mode change failing -> latch with torque off again (begin 1 + latch 1 = 2 each).
void test_mode_fail_latches(void) {
  fake.mode_ok = false;
  WheelPair wp(fake, mkCfg());
  TEST_ASSERT_FALSE(wp.begin(1u));
  TEST_ASSERT_FALSE(wp.alive());
  TEST_ASSERT_EQUAL_INT(F(Fault::WriteFail), F(wp.fault()));
  TEST_ASSERT_TRUE(fake.torque_off_calls[1] >= 2);
  TEST_ASSERT_TRUE(fake.torque_off_calls[2] >= 2);
  TEST_ASSERT_EQUAL_INT(0, fake.vel_calls);
}

// invalid config (NaN cap) -> refuse, zero bus traffic.
void test_invalid_config_refused(void) {
  WheelPair wp(fake, mkCfg(std::numeric_limits<float>::quiet_NaN()));
  TEST_ASSERT_FALSE(wp.begin(1u));
  TEST_ASSERT_FALSE(wp.alive());
  TEST_ASSERT_EQUAL_INT(0, fake.begin_calls);
  TEST_ASSERT_EQUAL_INT(0, fake.vel_calls);
}

// before begin: no write at all.
void test_write_before_begin(void) {
  WheelPair wp(fake, mkCfg());
  TEST_ASSERT_FALSE(wp.alive());
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_EQUAL_INT(0, fake.vel_calls);
}

// fresh begin() clears the latch.
void test_rebegin_clears_latch(void) {
  WheelPair wp(fake, mkCfg(1.0f, 1));
  TEST_ASSERT_TRUE(wp.begin(1u));
  fake.vel_ok[1] = false;
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));  // fail_max 1 -> latched
  TEST_ASSERT_FALSE(wp.alive());
  fake.vel_ok[1] = true;
  TEST_ASSERT_TRUE(wp.begin(1u));
  TEST_ASSERT_TRUE(wp.alive());
  TEST_ASSERT_EQUAL_INT(F(Fault::None), F(wp.fault()));
  TEST_ASSERT_TRUE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_EQUAL_INT32(21, fake.last_raw[1]);
}

// deliberate torqueOff(): confirmed, both ids, then latched.
void test_public_torque_off_latches(void) {
  WheelPair wp(fake, mkCfg());
  TEST_ASSERT_TRUE(wp.begin(1u));
  TEST_ASSERT_TRUE(wp.torqueOff());
  TEST_ASSERT_FALSE(wp.alive());
  TEST_ASSERT_EQUAL_INT(F(Fault::Latched), F(wp.fault()));
  TEST_ASSERT_EQUAL_INT(2, fake.torque_off_calls[1]);
  TEST_ASSERT_EQUAL_INT(2, fake.torque_off_calls[2]);
  TEST_ASSERT_FALSE(wp.writeRadS(0.5f, 0.5f));
  TEST_ASSERT_EQUAL_INT(2, fake.vel_calls);  // only the two begin() zero writes
}

void test_stop_writes_zero(void) {
  WheelPair wp(fake, mkCfg());
  TEST_ASSERT_TRUE(wp.begin(1u));
  TEST_ASSERT_TRUE(wp.writeRadS(0.5f, 0.5f));
  wp.stop();
  TEST_ASSERT_EQUAL_INT32(0, fake.last_raw[1]);
  TEST_ASSERT_EQUAL_INT32(0, fake.last_raw[2]);
}

// default setVelocityRawPair attempts BOTH ids even if the first fails.
void test_default_pair_tries_both(void) {
  fake.vel_ok[1] = false;
  TEST_ASSERT_FALSE(fake.setVelocityRawPair(1, 5, 2, 6));
  TEST_ASSERT_EQUAL_INT(2, fake.vel_calls);
  TEST_ASSERT_EQUAL_INT32(6, fake.last_raw[2]);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_begin_ok_and_order);
  RUN_TEST(test_zero_command);
  RUN_TEST(test_sign_per_wheel);
  RUN_TEST(test_clamp_max_rad_s);
  RUN_TEST(test_clamp_device_limit);
  RUN_TEST(test_limit_fallback_default);
  RUN_TEST(test_nan_inf_to_zero);
  RUN_TEST(test_latch_after_fail_max);
  RUN_TEST(test_fail_count_resets_on_success);
  RUN_TEST(test_unconfirmed_torque_off);
  RUN_TEST(test_ping_fail_no_velocity_write);
  RUN_TEST(test_readinfo_fail);
  RUN_TEST(test_mode_fail_latches);
  RUN_TEST(test_invalid_config_refused);
  RUN_TEST(test_write_before_begin);
  RUN_TEST(test_rebegin_clears_latch);
  RUN_TEST(test_public_torque_off_latches);
  RUN_TEST(test_stop_writes_zero);
  RUN_TEST(test_default_pair_tries_both);
  return UNITY_END();
}
