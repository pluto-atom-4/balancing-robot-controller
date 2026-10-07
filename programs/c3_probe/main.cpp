#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>    // LDF visibility (used through imu_mpu6050.h)
#include <Dynamixel2Arduino.h>   // LDF visibility (used through dxl_wheels.h)
#include <cstdint>
#include <cstdio>
#include <balance_core.h>
#include <balance_gains_generated.h>
#include <hal_iface.h>
#include <loop_stats.h>
#include <wheel_servo.h>
#include "../balance/config.h"
#include "../balance/imu_mpu6050.h"
#include "../balance/dxl_wheels.h"

// #43 c3_probe. Times IMU read, servo bus, controller step, heap, loop period on the real C3.
// SAFETY: torque stays OFF (confirmed by read-back) unless built with -D C3PROBE_TORQUE_ON.
// The only servo writes are goal velocity kGoalZeroRaw (0). No WheelPair, no writeRadS.
// Nothing here is validated on hardware until a human posts RESULTS on #43.

namespace {

constexpr uint32_t kSamples = 1000;
constexpr uint32_t kWarmup = 10;
constexpr uint32_t kBatch = 100;          // controller calls per timed window
constexpr uint32_t kMulAccIters = 1000;
constexpr uint32_t kMulAccRuns = 50;
constexpr uint32_t kPrintProbeN = 20;
constexpr uint32_t kLoopIters = 500;      // 10 s at kLoopUs
constexpr uint32_t kHostWaitMs = 5000;
constexpr uint32_t kRedumpMs = 15000;
constexpr uint32_t kNoBudget = 0xFFFFFFFFu;
constexpr int32_t kGoalZeroRaw = 0;       // the ONLY raw value this probe ever writes
constexpr float kProbeDt = 0.02f;         // = kLoopUs * 1e-6

Mpu6050Imu gImu;
DxlServo gServo;
balance::PidBalance gPid(balance_gains::kPidKp, balance_gains::kPidKi, balance_gains::kPidKd,
                         balance_gains::kPidOutLimit);
balance::LqrBalance gLqr(balance_gains::kLqrK[0], balance_gains::kLqrK[1],
                         balance_gains::kThetaRefRad, balance_gains::kOutLimit);

loop_stats::LoopStats gImuSt, gPingSt, gReadSt, gWr1St, gWr2St, gPidSt, gLqrSt, gSoftSt, gIntSt, gPrintSt, gLoopSt;
uint32_t gImuFail = 0, gPingFail = 0, gReadFail = 0, gWr1Fail = 0, gWr2Fail = 0, gLoopImuFail = 0;

hal::ImuSample gIn[8];          // filled at runtime so the compiler cannot fold the controller math
hal::ImuSample gS;
volatile float gSinkF = 0.0f;
volatile uint32_t gSinkU = 0;
int32_t gVel = 0;

bool gImuOk = false;
bool gUsableL = false, gUsableR = false, gUnsafe = false;
uint8_t gMode = 0;              // 0 = no servo (read-only), 1 = solo, 2 = pair
uint8_t gSoloId = 0;
uint8_t gTorqueHead = 1, gTorqueTail = 1;   // 0 = every pinged servo confirmed off, 1 = not confirmed
uint32_t gHeapStart = 0, gHeapEnd = 0, gHeapMin = 0;
char gBuild[32];

// loop run state (mirrors programs/balance/main.cpp)
uint32_t gNextUs = 0, gLastTickUs = 0, gIter = 0, gWorkMax = 0, gLastDumpMs = 0;
bool gHaveTick = false;
bool gDone = false;

#ifdef C3PROBE_TORQUE_ON
constexpr uint32_t kTorqueOnBuild = 1;
#else
constexpr uint32_t kTorqueOnBuild = 0;
#endif
#if defined(__riscv_flen)
constexpr uint32_t kRiscvFlen = __riscv_flen;
#else
constexpr uint32_t kRiscvFlen = 0;
#endif
#if defined(__riscv_float_abi_soft)
constexpr uint32_t kFloatAbiSoft = 1;
#else
constexpr uint32_t kFloatAbiSoft = 0;
#endif

// ---- output: one metric per line, exactly 4 tokens. Only place that prints metrics. ----
void emitWait() {
  const uint32_t t0 = millis();
  while (Serial.availableForWrite() < 64 && (millis() - t0) < 50) delay(1);
}
void emitU(const char* name, unsigned long v, const char* unit) {
  emitWait();
  Serial.printf("C3PROBE %s %lu %s\n", name, v, unit);
}
void emitS(const char* name, const char* v, const char* unit) {
  emitWait();
  Serial.printf("C3PROBE %s %s %s\n", name, v, unit);
}
void emitStats(const char* base, const loop_stats::LoopStats& st, const char* unit) {
  char n[48];
  snprintf(n, sizeof n, "%s_n", base);
  emitU(n, st.count, "count");
  if (st.count == 0) return;
  snprintf(n, sizeof n, "%s_min", base);
  emitU(n, st.min_us, unit);
  snprintf(n, sizeof n, "%s_mean", base);
  emitU(n, st.mean_us(), unit);
  snprintf(n, sizeof n, "%s_max", base);
  emitU(n, st.max_us, unit);
}

void dumpAll() {
  emitS("build", gBuild, "-");
  emitS("servo_family", gServo.family(), "-");
  emitU("servo_baud", kServoBaud, "baud");
  emitU("servo_io_timeout_ms", kDxlIoTimeoutMs, "ms");
  emitU("servo_mode", gMode, "-");
  emitU("servo_unsafe", gUnsafe ? 1 : 0, "-");
  emitU("torque_on_build", kTorqueOnBuild, "-");
  emitU("torque_state_head", gTorqueHead, "-");
  emitU("torque_state", gTorqueTail, "-");
  emitU("i2c_hz", kI2cHz, "Hz");
  emitU("cpu_mhz", getCpuFrequencyMhz(), "MHz");
  emitU("riscv_flen", kRiscvFlen, "-");
  emitU("riscv_float_abi_soft", kFloatAbiSoft, "-");
  emitU("imu_ok", gImuOk ? 1 : 0, "-");
  emitStats("imu_read_us", gImuSt, "us");
  emitU("imu_read_fail", gImuFail, "count");
  emitStats("servo_ping_us", gPingSt, "us");
  emitU("servo_ping_fail", gPingFail, "count");
  emitStats("servo_read_us", gReadSt, "us");
  emitU("servo_read_fail", gReadFail, "count");
  emitStats("servo_write1_us", gWr1St, "us");
  emitU("servo_write1_fail", gWr1Fail, "count");
  emitStats("servo_write2_us", gWr2St, "us");
  emitU("servo_write2_fail", gWr2Fail, "count");
  emitStats("pid_step_ns", gPidSt, "ns");
  emitStats("lqr_step_ns", gLqrSt, "ns");
  emitStats("softfloat_mulacc_us", gSoftSt, "us");
  emitStats("int_mulacc_us", gIntSt, "us");
  emitStats("print_line_us", gPrintSt, "us");
  emitU("loop_us", kLoopUs, "us");
  emitU("jitter_budget_us", kJitterBudgetUs, "us");
  emitU("loop_imu_fail", gLoopImuFail, "count");
  emitStats("loop_period_us", gLoopSt, "us");
  emitU("loop_over_budget", gLoopSt.over_budget, "count");
  emitU("work_us_max", gWorkMax, "us");
  emitU("heap_free_start", gHeapStart, "bytes");
  emitU("heap_free_end", gHeapEnd, "bytes");
  emitU("heap_min_free", gHeapMin, "bytes");
  emitU("done", 1, "-");
  Serial.flush();
  gLastDumpMs = millis();
}

void phase(const char* name) {
  emitS("phase", name, "start");
  Serial.flush();
  delay(20);  // let USB traffic settle before the next timed window
}

// ---- setup helpers ----
void initBuild() {
  snprintf(gBuild, sizeof gBuild, "%s_%s", __DATE__, __TIME__);
  for (size_t i = 0; gBuild[i] != '\0'; ++i) {
    if (gBuild[i] == ' ') gBuild[i] = '_';
  }
}

void initInputs() {
  static const float kPitchOff[8] = {0.0f, 0.01f, -0.01f, 0.05f, -0.05f, 0.2f, -0.2f, 0.4f};
  static const float kGyroY[8] = {0.0f, 0.5f, -0.5f, 1.0f, -1.0f, 2.0f, -2.0f, 3.0f};
  for (int i = 0; i < 8; ++i) {
    gIn[i] = hal::ImuSample();
    gIn[i].pitch_rad = balance_gains::kThetaRefRad + kPitchOff[i];
    gIn[i].gyro_rad_s[1] = kGyroY[i];
  }
}

// Torque-off FIRST, before IMU calibration. An id is usable only if it pings AND torque-off is confirmed.
void bringUpServo() {
  (void)gServo.begin(kServoBaud);
  const bool pL = gServo.ping(kServoIdLeft);
  const bool pR = gServo.ping(kServoIdRight);
  const bool offL = pL && gServo.torqueOff(kServoIdLeft);
  const bool offR = pR && gServo.torqueOff(kServoIdRight);
  gUsableL = offL;
  gUsableR = offR;
  gUnsafe = (pL && !offL) || (pR && !offR);
  gMode = static_cast<uint8_t>((gUsableL ? 1 : 0) + (gUsableR ? 1 : 0));
  gSoloId = gUsableL ? kServoIdLeft : kServoIdRight;
  gTorqueHead = gUnsafe ? 1 : 0;
}

#ifdef C3PROBE_TORQUE_ON
// Human opt-in only: torque ON at goal 0. Writes of nonzero velocity still never happen.
void torqueOnOptIn() {
  if (gUsableL && !gServo.enableVelocityMode(kServoIdLeft)) gUsableL = false;
  if (gUsableR && !gServo.enableVelocityMode(kServoIdRight)) gUsableR = false;
  gMode = static_cast<uint8_t>((gUsableL ? 1 : 0) + (gUsableR ? 1 : 0));
  gSoloId = gUsableL ? kServoIdLeft : kServoIdRight;
}
#endif

bool torqueOffAll() {
  bool ok = true;
  if (gUsableL) ok = gServo.torqueOff(kServoIdLeft) && ok;
  if (gUsableR) ok = gServo.torqueOff(kServoIdRight) && ok;
  return ok;
}

// ---- timed operations (no printing, no heap) ----
bool opImu() { return gImu.read(gS, kProbeDt); }
bool opPing() { return gServo.ping(gSoloId); }
bool opRead() { return gServo.presentVelocityRaw(gSoloId, gVel); }
bool opWrite1() { return gServo.setVelocityRaw(gSoloId, kGoalZeroRaw); }
bool opWrite2() {
  return gServo.setVelocityRawPair(kServoIdLeft, kGoalZeroRaw, kServoIdRight, kGoalZeroRaw);
}
bool servoTickWrite() {
  if (gMode == 2) return opWrite2();
  if (gMode == 1) return opWrite1();
  return false;
}

void timeOp(bool (*fn)(), loop_stats::LoopStats& st, uint32_t& fails) {
  for (uint32_t i = 0; i < kWarmup; ++i) (void)fn();
  for (uint32_t i = 0; i < kSamples; ++i) {
    const uint32_t t0 = micros();
    const bool ok = fn();
    const uint32_t t1 = micros();
    if (ok) {
      st.add(t1 - t0, kNoBudget);
    } else {
      ++fails;
    }
  }
}

void timePid() {
  for (uint32_t w = 0; w < kWarmup; ++w) gSinkF = gPid.step(gIn[w & 7], kProbeDt).cmd_rad_s;
  for (uint32_t w = 0; w < kSamples; ++w) {
    const uint32_t t0 = micros();
    for (uint32_t j = 0; j < kBatch; ++j) {
      gSinkF = gPid.step(gIn[j & 7], kProbeDt).cmd_rad_s;
      __asm__ __volatile__("" ::: "memory");
    }
    const uint32_t us = micros() - t0;
    gPidSt.add((us * 1000u) / kBatch, kNoBudget);
  }
}

void timeLqr() {
  for (uint32_t w = 0; w < kWarmup; ++w) gSinkF = gLqr.step(gIn[w & 7], kProbeDt).cmd_rad_s;
  for (uint32_t w = 0; w < kSamples; ++w) {
    const uint32_t t0 = micros();
    for (uint32_t j = 0; j < kBatch; ++j) {
      gSinkF = gLqr.step(gIn[j & 7], kProbeDt).cmd_rad_s;
      __asm__ __volatile__("" ::: "memory");
    }
    const uint32_t us = micros() - t0;
    gLqrSt.add((us * 1000u) / kBatch, kNoBudget);
  }
}

void timeMulAcc() {
  volatile float vx = 1.0001f;
  volatile float vy = 0.0001f;
  volatile uint32_t ux = 1664525u;
  volatile uint32_t uy = 1013904223u;
  for (uint32_t r = 0; r < kMulAccRuns; ++r) {
    const float x = vx;
    const float y = vy;
    float acc = 0.5f;
    const uint32_t t0 = micros();
    for (uint32_t i = 0; i < kMulAccIters; ++i) acc = acc * x + y;
    const uint32_t t1 = micros();
    gSinkF = acc;
    gSoftSt.add(t1 - t0, kNoBudget);
  }
  for (uint32_t r = 0; r < kMulAccRuns; ++r) {
    const uint32_t x = ux;
    const uint32_t y = uy;
    uint32_t acc = 1u;
    const uint32_t t0 = micros();
    for (uint32_t i = 0; i < kMulAccIters; ++i) acc = acc * x + y;
    const uint32_t t1 = micros();
    gSinkU = acc;
    gIntSt.add(t1 - t0, kNoBudget);
  }
}

// Cost of one balance-style [bal] line, so the human can budget prints. Output is a valid probe line.
void timePrint() {
  for (uint32_t i = 0; i < kPrintProbeN; ++i) {
    emitWait();
    const uint32_t t0 = micros();
    Serial.printf("C3PROBE print_probe %lu -\n", static_cast<unsigned long>(i));
    const uint32_t t1 = micros();
    gPrintSt.add(t1 - t0, kNoBudget);
  }
  Serial.flush();
  delay(20);
}

// One paced tick, same shape as balance controlTick: IMU read + servo goal-0 write + one LQR step (never applied).
void loopTick(uint32_t now) {
  const uint32_t w0 = micros();
  const uint32_t period = now - gLastTickUs;
  const bool first = !gHaveTick || period == 0;
  gLastTickUs = now;
  gHaveTick = true;
  const float dt = static_cast<float>(first ? kLoopUs : period) * 1e-6f;
  bool imuOk = false;
  if (gImuOk) {
    imuOk = gImu.read(gS, dt);
    if (!imuOk) ++gLoopImuFail;
  }
  (void)servoTickWrite();
  gSinkF = gLqr.step(imuOk ? gS : gIn[0], dt).cmd_rad_s;
  const uint32_t work = micros() - w0;
  if (work > gWorkMax) gWorkMax = work;
  if (!first) gLoopSt.add(period, kJitterBudgetUs);
  ++gIter;
}

void finish() {
  gHeapEnd = ESP.getFreeHeap();
  gHeapMin = ESP.getMinFreeHeap();
  const bool off = torqueOffAll();   // idempotent; read-back confirms
  gTorqueTail = (off && !gUnsafe) ? 0 : 1;
  gDone = true;
  dumpAll();
}

}  // namespace

void setup() {
  Serial.begin(kUsbSerialBaud);
  bringUpServo();                      // torque-off first, before any wait
  const uint32_t w0 = millis();
  while (!Serial && (millis() - w0) < kHostWaitMs) delay(10);
  initBuild();
  initInputs();
  gHeapStart = ESP.getFreeHeap();
  emitU("probe_start", 1, "-");
  emitU("torque_state_head", gTorqueHead, "-");
  if (gUnsafe) emitU("servo_unsafe", 1, "REMOVE_SERVO_POWER");

  phase("imu_calibrate");              // ~1.2 s, robot must be still
  if (gImu.begin(D4, D5, kI2cHz)) gImuOk = gImu.calibrateGyroBias();

#ifdef C3PROBE_TORQUE_ON
  emitU("torque_on_build", 1, "-");
  torqueOnOptIn();
#endif

  if (gImuOk) { phase("imu_read"); timeOp(opImu, gImuSt, gImuFail); }
  if (gMode > 0) {
    phase("servo_ping");   timeOp(opPing, gPingSt, gPingFail);
    phase("servo_read");   timeOp(opRead, gReadSt, gReadFail);
    phase("servo_write1"); timeOp(opWrite1, gWr1St, gWr1Fail);
    if (gMode == 2) { phase("servo_write2"); timeOp(opWrite2, gWr2St, gWr2Fail); }
  }
  phase("pid_step");  timePid();
  phase("lqr_step");  timeLqr();
  phase("mulacc");    timeMulAcc();
  phase("print");     timePrint();

  phase("loop_run");                   // ~10 s, no prints inside
  gLoopSt.reset();
  gHaveTick = false;
  gIter = 0;
  gNextUs = micros() + kLoopUs;
}

void loop() {
  if (gDone) {
    if (millis() - gLastDumpMs >= kRedumpMs) dumpAll();
    return;
  }
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - gNextUs) < 0) return;
  gNextUs += kLoopUs;
  if (static_cast<int32_t>(now - gNextUs) >= 0) gNextUs = now + kLoopUs;  // overran a whole period: resync
  loopTick(now);
  if (gIter >= kLoopIters) finish();
}
