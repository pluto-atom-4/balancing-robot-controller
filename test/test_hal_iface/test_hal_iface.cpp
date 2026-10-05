#include <unity.h>
#include <hal_iface.h>
#include <cstddef>
#include <cstdint>
#include <type_traits>

static_assert(sizeof(float) == 4, "float must be 32-bit");
static_assert(std::is_trivially_copyable<hal::ImuSample>::value,
              "ImuSample must be trivially copyable");
static_assert(std::is_trivially_copyable<hal::EncoderSample>::value,
              "EncoderSample must be trivially copyable");
static_assert(std::is_standard_layout<hal::ImuSample>::value,
              "ImuSample must be standard layout");
static_assert(sizeof(hal::ImuSample) == 28, "ImuSample size drift");
static_assert(sizeof(hal::EncoderSample) == 8, "EncoderSample size drift");
static_assert(offsetof(hal::ImuSample, t_s) == 0, "field order");
static_assert(offsetof(hal::ImuSample, roll_rad) == 4, "field order");
static_assert(offsetof(hal::ImuSample, pitch_rad) == 8, "field order");
static_assert(offsetof(hal::ImuSample, yaw_rad) == 12, "field order");
static_assert(offsetof(hal::ImuSample, gyro_rad_s) == 16, "field order");
static_assert(offsetof(hal::EncoderSample, left_rad) == 0, "field order");
static_assert(offsetof(hal::EncoderSample, right_rad) == 4, "field order");

namespace {

// Scripted stub: proves hal::Hal is implementable and returns scripted values.
class FakeHal : public hal::Hal {
 public:
  bool imu_ok = true;
  bool enc_ok = true;
  float last_left = 0.0f;
  float last_right = 0.0f;
  float t = 0.0f;
  float next_dt = 0.02f;

  bool read_imu(hal::ImuSample& out) override {
    if (!imu_ok) return false;
    out.t_s = t;
    out.roll_rad = 0.01f;
    out.pitch_rad = -0.1086f;
    out.yaw_rad = 0.03f;
    out.gyro_rad_s[0] = 0.1f;
    out.gyro_rad_s[1] = 0.2f;
    out.gyro_rad_s[2] = 0.3f;
    return true;
  }
  bool read_encoders(hal::EncoderSample& out) override {
    if (!enc_ok) return false;
    out.left_rad = 1.5f;
    out.right_rad = -2.5f;
    return true;
  }
  void write_wheel_velocity(float left_rad_s, float right_rad_s) override {
    last_left = left_rad_s;
    last_right = right_rad_s;
  }
  float now_s() override { return t; }
  float wait_next_tick() override {
    if (next_dt >= 0.0f) t += next_dt;
    return next_dt;
  }
};

}  // namespace

void setUp(void) {}

void tearDown(void) {}

// Test 1: contract version stamp must equal HAL_CONTRACT_VERSION in hal.py
void test_contract_version(void) {
  TEST_ASSERT_EQUAL_UINT32(1, hal::kContractVersion);
}

// Test 2: FakeHal IMU returns scripted values through the base interface
void test_fake_imu_scripted(void) {
  FakeHal fake;
  hal::Hal& h = fake;
  hal::ImuSample s{};
  TEST_ASSERT_TRUE(h.read_imu(s));
  TEST_ASSERT_EQUAL_FLOAT(0.01f, s.roll_rad);
  TEST_ASSERT_EQUAL_FLOAT(-0.1086f, s.pitch_rad);
  TEST_ASSERT_EQUAL_FLOAT(0.03f, s.yaw_rad);
  TEST_ASSERT_EQUAL_FLOAT(0.2f, s.gyro_rad_s[1]);
}

// Test 3: false return signals a fault (HalFault in Python)
void test_fault_returns_false(void) {
  FakeHal fake;
  fake.imu_ok = false;
  fake.enc_ok = false;
  hal::Hal& h = fake;
  hal::ImuSample s{};
  hal::EncoderSample e{};
  TEST_ASSERT_FALSE(h.read_imu(s));
  TEST_ASSERT_FALSE(h.read_encoders(e));
}

// Test 4: encoders and wheel write round trip
void test_encoders_and_write(void) {
  FakeHal fake;
  hal::Hal& h = fake;
  hal::EncoderSample e{};
  TEST_ASSERT_TRUE(h.read_encoders(e));
  TEST_ASSERT_EQUAL_FLOAT(1.5f, e.left_rad);
  TEST_ASSERT_EQUAL_FLOAT(-2.5f, e.right_rad);
  h.write_wheel_velocity(3.0f, -4.0f);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, fake.last_left);
  TEST_ASSERT_EQUAL_FLOAT(-4.0f, fake.last_right);
}

// Test 5: wait_next_tick returns dt, negative means stop
void test_wait_next_tick_semantics(void) {
  FakeHal fake;
  hal::Hal& h = fake;
  TEST_ASSERT_EQUAL_FLOAT(0.02f, h.wait_next_tick());
  TEST_ASSERT_EQUAL_FLOAT(0.02f, h.now_s());
  fake.next_dt = -1.0f;
  TEST_ASSERT_TRUE(h.wait_next_tick() < 0.0f);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_contract_version);
  RUN_TEST(test_fake_imu_scripted);
  RUN_TEST(test_fault_returns_false);
  RUN_TEST(test_encoders_and_write);
  RUN_TEST(test_wait_next_tick_semantics);
  return UNITY_END();
}
