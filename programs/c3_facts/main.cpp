// c3_facts (#44): static IMU facts + servo facts + opt-in capped velocity ramp. Bench tool.
// SAFETY: servo OFF the robot, held down, no wheel attached. Torque off at boot and on every exit.
// The ramp exists ONLY in builds with -D C3FACTS_RAMP and still needs a typed GO. Nothing validated on hardware.
// C++11-valid. Relative includes only (no -I).

#include <Arduino.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <balance_gains_generated.h>
#include <hal_iface.h>
#include <imu_math.h>
#include <ramp_guard.h>
#include <tilt.h>
#include <wheel_servo.h>
#include "../balance/config.h"
#include "../balance/imu_mpu6050.h"
#include "../balance/dxl_wheels.h"
#include "facts_io.h"

using facts_io::emitBool;
using facts_io::emitFloat;
using facts_io::emitInt;
using facts_io::emitStr;
using facts_io::prompt;

static constexpr uint32_t kBaudTryOrder[2] = {kServoBaud, kServoBaudRequested};  // proven first, 2M untested
static constexpr uint16_t kStillSamples = 500;
static constexpr int32_t kRampCapPct = 20;       // lib refuses > 25
static constexpr int32_t kGoalZeroRaw = 0;
static constexpr uint32_t kRampStepMs = 50;
static constexpr uint32_t kRampRowMs = 250;
static constexpr uint32_t kCountdownS = 5;
static constexpr uint32_t kTiltPromptMs = 15000;
static constexpr uint32_t kTiltCaptureMs = 5000;
static constexpr uint32_t kTiltHoldMs = 1000;
static constexpr float kGravity = 9.80665f;

static Mpu6050Imu imu;
static DxlServo servo;
static uint32_t gBaud = 0;           // baud where a servo answered at boot; 0 = none
static bool gBootPing[2] = {false, false};
static bool gBootOff[2] = {false, false};
static bool gTorqueUnconfirmed = false;  // loop() keeps retrying torque-off while true

static const uint8_t kIds[2] = {kServoIdLeft, kServoIdRight};

// ---- Boot: torque off FIRST, at every baud, before IMU or anything else ----
static void bootTorqueOff() {
  for (uint8_t b = 0; b < 2; ++b) {
    (void)servo.begin(kBaudTryOrder[b]);
    bool any = false;
    for (uint8_t i = 0; i < 2; ++i) {
      if (servo.ping(kIds[i])) {
        gBootPing[i] = true;
        gBootOff[i] = servo.torqueOff(kIds[i]);
        any = true;
      }
    }
    if (any) {
      gBaud = kBaudTryOrder[b];
      return;
    }
  }
}

static void emitBoot() {
  emitStr("build_date", __DATE__ " " __TIME__, "-");
  emitInt("cpu_mhz", static_cast<long>(getCpuFrequencyMhz()), "MHz");
  bool reachable = false;
  bool allOff = true;
  for (uint8_t i = 0; i < 2; ++i) {
    if (gBootPing[i]) {
      reachable = true;
      if (!gBootOff[i]) allOff = false;
    }
  }
  emitBool("servo_reachable_at_boot", reachable);
  emitBool("torque_off_at_boot", reachable && allOff);
  if (reachable && !allOff) {
    gTorqueUnconfirmed = true;
    prompt("TORQUE OFF NOT CONFIRMED AT BOOT: REMOVE SERVO POWER");
  }
}

// ---- IMU still phase ----
static bool gImuOk = false;
static float gRestTiltDeg = 0.0f;

static void imuStill() {
  gImuOk = imu.begin(D4, D5, kI2cHz);
  emitBool("imu_began", gImuOk);
  if (!gImuOk) return;
  prompt("keep the robot STILL and FLAT, sampling");
  const bool cal = imu.calibrateGyroBias(kStillSamples);
  emitBool("gyro_bias_calibrated", cal);  // 0 = outside kMaxBiasRadS/kMaxBiasVar; values still printed
  static const char* const kBias[3] = {"gyro_bias_x", "gyro_bias_y", "gyro_bias_z"};
  static const char* const kStd[3] = {"gyro_bias_x_std", "gyro_bias_y_std", "gyro_bias_z_std"};
  for (int k = 0; k < 3; ++k) {
    emitFloat(kBias[k], imu.biasRadS(k), "rad_s", 6);
    emitFloat(kStd[k], imu.biasStdRadS(k), "rad_s", 6);
  }
  imu_math::BiasAccumulator ax, ay, az;
  uint16_t good = 0;
  for (uint16_t i = 0; i < kStillSamples; ++i) {
    float a[3], g[3];
    if (imu.readRaw(a, g)) {
      ax.add(a[0]);
      ay.add(a[1]);
      az.add(a[2]);
      ++good;
    }
    delay(2);
  }
  emitInt("accel_samples_ok", good, "count");
  if (good < kStillSamples / 2) {
    gImuOk = false;
    return;
  }
  emitFloat("accel_x", ax.mean / kGravity, "g", 5);
  emitFloat("accel_y", ay.mean / kGravity, "g", 5);
  emitFloat("accel_z", az.mean / kGravity, "g", 5);
  emitFloat("accel_x_std", ax.stddev() / kGravity, "g", 5);
  emitFloat("accel_y_std", ay.stddev() / kGravity, "g", 5);
  emitFloat("accel_z_std", az.stddev() / kGravity, "g", 5);
  const float norm = std::sqrt(ax.mean * ax.mean + ay.mean * ay.mean + az.mean * az.mean) / kGravity;
  emitFloat("accel_norm", norm, "g", 4);
  emitBool("accel_norm_ok", norm > 0.9f && norm < 1.1f);
  gRestTiltDeg = tilt::accelPitchDeg(ax.mean, ay.mean, az.mean);
  const float restHal = imu_math::pitchRadFromDeg(gRestTiltDeg, kPitchSign);
  emitFloat("pitch_rest_tilt_deg", gRestTiltDeg, "deg", 3);
  emitFloat("pitch_rest_hal_rad", restHal, "rad", 5);
  emitFloat("pitch_rest_hal_deg", restHal * tilt::kRadToDeg, "deg", 3);
  emitFloat("pitch_sign_used", kPitchSign, "-", 1);
  emitFloat("gyro_pitch_sign_used", kGyroPitchSign, "-", 1);
  emitFloat("theta_ref_sim", balance_gains::kThetaRefRad, "rad_SIM_UNVERIFIED", 5);
  emitFloat("pitch_minus_theta_ref", restHal - balance_gains::kThetaRefRad, "rad", 5);
}

// ---- Optional forward-tilt phase (typed 't') ----
static void tiltPhase() {
  if (!gImuOk) return;
  prompt("type t + Enter within 15 s for the forward-tilt capture, else it is skipped");
  facts_io::drainSerial();
  bool want = false;
  const uint32_t w0 = millis();
  while (!want && millis() - w0 < kTiltPromptMs) {
    while (Serial.available() > 0) {
      const int c = Serial.read();
      if (c == 't' || c == 'T') want = true;
    }
    delay(10);
  }
  emitBool("tilt_phase_run", want);
  if (!want) return;
  prompt("tilt the body FORWARD about 20 deg (direction per the pitch comment in hal_iface.h) then hold still. 5 s capture starts NOW");
  imu_math::BiasAccumulator ax, ay, az;
  float integral = 0.0f;
  uint32_t prev = micros();
  const uint32_t t0 = millis();
  while (millis() - t0 < kTiltCaptureMs) {
    float a[3], g[3];
    const uint32_t now = micros();
    if (!imu.readRaw(a, g)) {
      emitBool("tilt_capture_ok", false);
      return;
    }
    const float dt = static_cast<float>(now - prev) * 1e-6f;
    prev = now;
    integral += (g[1] - imu.biasRadS(1)) * dt;
    if (millis() - t0 >= kTiltCaptureMs - kTiltHoldMs) {
      ax.add(a[0]);
      ay.add(a[1]);
      az.add(a[2]);
    }
    delay(2);
  }
  emitBool("tilt_capture_ok", true);
  const float endDeg = tilt::accelPitchDeg(ax.mean, ay.mean, az.mean);
  const float deltaTiltRad = (endDeg - gRestTiltDeg) * imu_math::kDegToRad;
  emitFloat("pitch_forward_tilt_deg", endDeg, "deg", 3);
  emitFloat("pitch_forward_hal_rad", imu_math::pitchRadFromDeg(endDeg, kPitchSign), "rad", 5);
  emitFloat("pitch_forward_delta_tilt_rad", deltaTiltRad, "rad", 5);
  emitFloat("gyro_y_integral_rad", integral, "rad", 5);
  // HINT only; the human decides kPitchSign and kGyroPitchSign. -1 = not decidable (motion too small).
  const bool big = std::fabs(deltaTiltRad) >= 0.2f && std::fabs(integral) >= 0.2f;
  emitInt("gyro_sign_consistent_candidate",
          big ? (((kGyroPitchSign * integral > 0.0f) == (deltaTiltRad > 0.0f)) ? 1 : 0) : -1, "-");
}

// ---- Servo facts ----
struct IdFacts {
  bool ping;
  bool info_ok;
  wheel_servo::DeviceInfo info;
  bool torque_ok;
  int32_t torque;
  bool mode_ok;
  int32_t mode;
};
static IdFacts gFacts[2];

static void servoFacts() {
  uint32_t workBaud = 0;
  for (uint8_t b = 0; b < 2; ++b) {
    (void)servo.begin(kBaudTryOrder[b]);
    const bool ok = servo.ping(kServoIdLeft);
    char key[32];
    std::snprintf(key, sizeof(key), "baud_%lu_ok", static_cast<unsigned long>(kBaudTryOrder[b]));
    emitBool(key, ok);
    if (ok && workBaud == 0) workBaud = kBaudTryOrder[b];
  }
  emitInt("baud_used", static_cast<long>(workBaud), "baud");
  emitBool("baud_requested_2m_untested", true);
  if (workBaud == 0) {
    prompt("no baud answered for id 1");
    return;
  }
  (void)servo.begin(workBaud);  // back to the working rate (a repeated begin is UNVERIFIED on hardware)
  gBaud = workBaud;
  emitStr("servo_family_assumed", servo.family(), "-");
  emitFloat("assumed_rpm_per_raw", servo.units().rpm_per_raw, "rpm", 4);
  emitInt("assumed_vel_limit_raw", servo.units().default_limit_raw, "raw");
  for (uint8_t i = 0; i < 2; ++i) {
    IdFacts& f = gFacts[i];
    f.ping = false;
    f.info_ok = false;
    f.info = wheel_servo::DeviceInfo();
    f.torque_ok = false;
    f.torque = -1;
    f.mode_ok = false;
    f.mode = -1;
    char key[40];
    f.ping = servo.ping(kIds[i]);
    std::snprintf(key, sizeof(key), "servo%u_ping", static_cast<unsigned>(kIds[i]));
    emitBool(key, f.ping);
    if (!f.ping) continue;  // missing id is reported, never fatal
    f.info_ok = servo.readInfo(kIds[i], f.info);
    std::snprintf(key, sizeof(key), "servo%u_model_number", static_cast<unsigned>(kIds[i]));
    emitInt(key, f.info.model_number, "-");
    std::snprintf(key, sizeof(key), "servo%u_model_name", static_cast<unsigned>(kIds[i]));
    emitStr(key, facts_io::modelName(f.info.model_number), "-");
    std::snprintf(key, sizeof(key), "servo%u_vel_limit_device", static_cast<unsigned>(kIds[i]));
    emitInt(key, f.info.have_vel_limit ? f.info.vel_limit_raw : -1, "raw");
    std::snprintf(key, sizeof(key), "servo%u_vel_limit_matches_assumed", static_cast<unsigned>(kIds[i]));
    emitBool(key, f.info.have_vel_limit && f.info.vel_limit_raw == servo.units().default_limit_raw);
    f.mode_ok = servo.readOperatingMode(kIds[i], f.mode);
    std::snprintf(key, sizeof(key), "servo%u_operating_mode", static_cast<unsigned>(kIds[i]));
    emitInt(key, f.mode_ok ? f.mode : -1, "-");
    f.torque_ok = servo.readTorqueEnable(kIds[i], f.torque);
    std::snprintf(key, sizeof(key), "servo%u_torque_state", static_cast<unsigned>(kIds[i]));
    emitInt(key, f.torque_ok ? f.torque : -1, "-");
    int32_t pv = 0;
    const bool pok = servo.presentVelocityRaw(kIds[i], pv);
    std::snprintf(key, sizeof(key), "servo%u_present_velocity_rest", static_cast<unsigned>(kIds[i]));
    emitInt(key, pok ? pv : 0, pok ? "raw" : "raw_READ_FAILED");
  }
}

// ---- Ramp: compiled only with -D C3FACTS_RAMP ----
#ifdef C3FACTS_RAMP

static const char* abortName(ramp_guard::Abort a) {
  switch (a) {
    case ramp_guard::Abort::None: return "complete";
    case ramp_guard::Abort::SerialByte: return "serial_byte_or_host_gone";
    case ramp_guard::Abort::Timeout: return "hard_timeout";
    case ramp_guard::Abort::WriteFail: return "write_fail";
    case ramp_guard::Abort::ReadFail: return "read_fail";
    case ramp_guard::Abort::Overspeed: return "overspeed";
  }
  return "unknown";
}

// EVERY ramp exit goes through here: goal 0 (best effort) then torque off, up to 3 confirmed attempts.
static bool finishServo(uint8_t id) {
  (void)servo.setVelocityRaw(id, kGoalZeroRaw);
  for (uint8_t i = 0; i < 3; ++i) {
    if (servo.torqueOff(id)) return true;
  }
  return false;
}

static void runRamp(uint8_t id, int32_t cap) {
  emitInt("ramp_cap_raw", cap, "raw");
  emitFloat("ramp_cap_rad_s", dxl_units::rawToRadS(servo.units(), cap), "rad_s", 3);
  if (!servo.enableVelocityMode(id)) {  // writes the EEPROM mode ONLY if not velocity (gated by caller)
    emitBool("ramp_enable_ok", false);
    gTorqueUnconfirmed = !finishServo(id);
    emitBool("torque_off_after_ramp", !gTorqueUnconfirmed);
    return;
  }
  emitBool("ramp_enable_ok", true);
  ramp_guard::Abort why = ramp_guard::Abort::None;
  const uint32_t t0 = millis();
  uint32_t lastRow = 0;
  bool firstRow = true;
  for (;;) {
    const uint32_t t = millis() - t0;
    if (ramp_guard::rampDone(t)) break;
    const bool byteOrGone = (facts_io::drainSerial() > 0) || !Serial;
    ramp_guard::StepInput pre = {t, byteOrGone, true, true, 0, cap};
    why = ramp_guard::check(pre);  // BEFORE any write: serial byte or timeout stops without a new goal
    if (why != ramp_guard::Abort::None) break;
    const int32_t goal = ramp_guard::goalAt(t, cap);
    const bool wok = servo.setVelocityRaw(id, goal);
    int32_t present = 0;
    const bool rok = servo.presentVelocityRaw(id, present);
    ramp_guard::StepInput post = {t, false, wok, rok, present, cap};
    why = ramp_guard::check(post);
    if (why != ramp_guard::Abort::None) break;
    if (firstRow || t - lastRow >= kRampRowMs) {
      char v[40];
      std::snprintf(v, sizeof(v), "%lu,%ld,%ld", static_cast<unsigned long>(t), static_cast<long>(goal),
                    static_cast<long>(present));
      emitStr("ramp_row", v, "ms_raw_raw");
      lastRow = t;
      firstRow = false;
    }
    delay(kRampStepMs);
  }
  const bool off = finishServo(id);
  gTorqueUnconfirmed = !off;
  emitStr("ramp_end_reason", abortName(why), "-");
  emitBool("torque_off_after_ramp", off);
  int32_t tq = -1;
  emitInt("torque_state_final", servo.readTorqueEnable(id, tq) ? tq : -1, "-");
  if (!off) prompt("TORQUE OFF NOT CONFIRMED: REMOVE SERVO POWER");
}

static void offerRamp() {
  const uint8_t id = kServoIdLeft;
  const IdFacts& f = gFacts[0];
  ramp_guard::Pre pre;
  pre.ping_ok = f.ping;
  pre.model_known = f.info_ok && f.info.model_number == facts_io::kModelXl330M077;
  pre.limit_from_device = f.info_ok && f.info.have_vel_limit;
  pre.limit_raw = pre.limit_from_device ? f.info.vel_limit_raw : 0;
  pre.id_is_bench = (id == kServoIdLeft);
  pre.host_connected = static_cast<bool>(Serial);
#ifdef C3FACTS_ALLOW_MODE_WRITE
  pre.mode_ok = true;  // human accepted a possible EEPROM Operating Mode write
#else
  pre.mode_ok = f.mode_ok && f.mode == static_cast<int32_t>(kDxlOpModeVelocity);
#endif
  pre.torque_off_confirmed = f.ping && servo.torqueOff(id);  // re-confirmed now, by read-back
  emitBool("ramp_pre_ping", pre.ping_ok);
  emitBool("ramp_pre_model_1190", pre.model_known);
  emitBool("ramp_pre_limit_from_device", pre.limit_from_device);
  emitBool("ramp_pre_mode_ok", pre.mode_ok);
  emitBool("ramp_pre_host_connected", pre.host_connected);
  emitBool("ramp_pre_torque_off", pre.torque_off_confirmed);
  const int32_t cap = ramp_guard::capFromLimit(pre.limit_raw, kRampCapPct);
  if (!ramp_guard::rampAllowed(pre, kRampCapPct)) {
    emitBool("ramp_refused", true);
    return;
  }
  prompt("WARNING: servo must be OFF the robot, HELD DOWN, no wheel attached.");
  prompt("Type GO + Enter within 60 s to start a capped velocity ramp (about 15 s). Any key during the ramp aborts.");
  facts_io::drainSerial();
  ramp_guard::GoParser parser;
  bool go = false;
  const uint32_t start = millis();
  while (!go && !ramp_guard::windowExpired(millis(), start, ramp_guard::kGoWindowMs)) {
    while (Serial.available() > 0 && !go) {
      const ramp_guard::GoResult r = parser.feed(static_cast<char>(Serial.read()));
      if (r == ramp_guard::GoResult::Go) go = true;
      else if (r == ramp_guard::GoResult::Reject) prompt("not GO, still waiting");
    }
    delay(10);
  }
  if (!go) {
    emitStr("ramp_skipped", "no_go_in_window", "-");
    return;
  }
  // Terminals often send CRLF: the GO parser fires on the CR, so drop the leftover LF now, otherwise the
  // countdown below would treat it as an abort key. Keys typed AFTER this point still abort.
  facts_io::drainSerial();
  for (uint32_t s = kCountdownS; s > 0; --s) {
    char m[32];
    std::snprintf(m, sizeof(m), "ramp in %lu, any key aborts", static_cast<unsigned long>(s));
    prompt(m);
    delay(1000);
    if (facts_io::drainSerial() > 0 || !Serial) {
      emitStr("ramp_skipped", "aborted_in_countdown", "-");
      return;
    }
  }
  if (!servo.ping(id) || !servo.torqueOff(id)) {
    emitStr("ramp_skipped", "reconfirm_failed", "-");
    return;
  }
  runRamp(id, cap);
}

#endif  // C3FACTS_RAMP

void setup() {
  Serial.begin(kUsbSerialBaud);
  bootTorqueOff();  // FIRST servo action, before IMU, before any print
  const uint32_t w0 = millis();
  while (!Serial && millis() - w0 < 5000) delay(10);
  emitBoot();
  imuStill();
  tiltPhase();
  servoFacts();
#ifdef C3FACTS_RAMP
  emitBool("ramp_compiled", true);
  offerRamp();
#else
  emitBool("ramp_compiled", false);
  prompt("ramp not compiled; rebuild with PLATFORMIO_BUILD_FLAGS=\"-D C3FACTS_RAMP\" to offer it");
#endif
  emitBool("done", true);
}

void loop() {
  // Idle. If torque-off was ever unconfirmed, keep retrying it (writes nothing else).
  if (gTorqueUnconfirmed) {
    bool allOff = true;
    for (uint8_t i = 0; i < 2; ++i) {
      if (gFacts[i].ping || gBootPing[i]) {
        if (!servo.torqueOff(kIds[i])) allOff = false;
      }
    }
    if (allOff) {
      gTorqueUnconfirmed = false;
      emitBool("torque_off_recovered", true);
    }
  }
  delay(1000);
}
