#include <Arduino.h>

#if defined(BALANCE_CONTROLLER_LQR) && defined(BALANCE_CONTROLLER_PID)
#error "define only one of BALANCE_CONTROLLER_LQR or BALANCE_CONTROLLER_PID"
#elif !defined(BALANCE_CONTROLLER_LQR) && !defined(BALANCE_CONTROLLER_PID)
#error "define BALANCE_CONTROLLER_LQR or BALANCE_CONTROLLER_PID"
#endif

// Balance control loop (#32): IMU -> controller -> WheelPair, with a latched supervisor.
// PLANNED/UNVERIFIED: nothing here is validated on hardware. PID sign is UNVERIFIED (freecad-workspace#359).
// LQR gains are SIM-derived. Servo family is hidden behind IWheelServo; no Dynamixel calls here.

#include <cmath>
#include <balance_core.h>
#include <balance_gains_generated.h>
#include <balance_supervisor.h>
#include <hal_iface.h>
#include <loop_stats.h>
#include <wheel_servo.h>
#include "imu_mpu6050.h"
#include "config.h"
#include "dxl_wheels.h"

static_assert(balance_gains::kContractVersion == hal::kContractVersion,
              "generated gains contract version != hal_iface contract version");
static_assert(kStartupLeanGateRad > 0.0f && kStartupLeanGateRad < kTiltCutoffRad, "gate must be < cutoff");
static_assert(kStallUs > 4u * kLoopUs, "stall threshold must exceed 3 tolerated missed updates (#345)");

namespace bsup = balance_supervisor;

#if defined(BALANCE_CONTROLLER_LQR)
static balance::LqrBalance ctrl(balance_gains::kLqrK[0], balance_gains::kLqrK[1],
                                balance_gains::kThetaRefRad, balance_gains::kOutLimit);
static constexpr const char* kCtrlName = "LQR";
static constexpr float kCenterRad = balance_gains::kThetaRefRad;  // controller setpoint
#else
static balance::PidBalance ctrl(balance_gains::kPidKp, balance_gains::kPidKi, balance_gains::kPidKd,
                                balance_gains::kPidOutLimit);
static constexpr const char* kCtrlName = "PID";
static constexpr float kCenterRad = 0.0f;  // PID error = -pitch: setpoint is 0, THETA_REF unused
#endif

static constexpr uint32_t kBalPrintUs = 100000;     // 10 Hz [bal]
static constexpr uint32_t kLoopPrintUs = 1000000;   // 1 Hz [loop]
static constexpr uint32_t kRetryMs = 3000;          // IMU bring-up / calibration retry
static constexpr uint32_t kTorqueRetryMs = 1000;
static constexpr uint8_t kTorqueRetryMax = 5;

static Mpu6050Imu imu;
static bool imuBegun = false;
static hal::ImuSample sample = {};

static DxlServo servoBus;
static wheel_servo::WheelPair wheels(
    servoBus,
    wheel_servo::WheelPairConfig{kServoIdLeft, kServoIdRight, kLeftWheelSign, kRightWheelSign,
                                 kMaxWheelRadS, kServoFailMax});

static bsup::Supervisor sup(bsup::Config{kCenterRad, kTiltCutoffRad, kStartupLeanGateRad,
                                         kLeanGateHoldTicks, kImuFailMax, kStallUs});
static loop_stats::LoopStats stats;

static uint32_t nextUs = 0;
static uint32_t lastTickUs = 0;
static bool haveTick = false;
static uint32_t lastBalPrintUs = 0;
static uint32_t lastLoopPrintUs = 0;
static uint32_t lastTryMs = 0;
static uint32_t lastTorqueTryMs = 0;
static bool latchAnnounced = false;
static bool torqueOffConfirmed = false;
static uint8_t torqueTries = 0;

// Torque-off is the FIRST servo action (before IMU begin and the blocking calibration). No printing here.
static bool startWheels() { return wheels.begin(kServoBaud); }

static void reportWheels(bool ok) {
  if (ok) {
    Serial.printf("[wheel] %s ready baud=%lu ids=%u/%u torque on, goal 0 (UNVERIFIED on hardware)\n",
                  servoBus.family(), static_cast<unsigned long>(kServoBaud),
                  static_cast<unsigned>(kServoIdLeft), static_cast<unsigned>(kServoIdRight));
  } else {
    const bool unconfirmed = (wheels.fault() == wheel_servo::Fault::TorqueOffUnconfirmed);
    Serial.printf("[wheel] startup FAILED fault=%d, torque off, read-only%s\n",
                  static_cast<int>(wheels.fault()), unconfirmed ? " (TORQUE OFF NOT CONFIRMED)" : "");
  }
}

static bool tryCalibrate() {
  Serial.println("[imu] calibrating gyro bias, keep robot still");
  const bool ok = imu.calibrateGyroBias();
  if (ok) {
    Serial.printf("[imu] bias ok y=%.5f rad/s (UNVERIFIED on hardware)\n", imu.biasRadS(1));
  } else {
    Serial.printf("[imu] bias FAILED y=%.5f std=%.5f rad/s\n", imu.biasRadS(1), imu.biasStdRadS(1));
  }
  return ok;
}

// After a blocking calibration the first tick gap is meaningless: restart timing and stats.
static void onCalibrated() {
  sup.onCalibration(true);
  stats.reset();
  haveTick = false;
  const uint32_t now = micros();
  nextUs = now + kLoopUs;
  lastBalPrintUs = now;
  lastLoopPrintUs = now;
  Serial.printf("[bal] %s waiting for lean gate (|pitch-center|<=%.3f rad)\n", kCtrlName,
                static_cast<double>(kStartupLeanGateRad));
}

// Every kill path ends here: WheelPair::torqueOff() (latched). Torque-off BEFORE printing. The loop keeps running.
static void enforceLatch() {
  if (!sup.latched() || torqueOffConfirmed) return;
  const uint32_t nowMs = millis();
  if (latchAnnounced && (nowMs - lastTorqueTryMs) < kTorqueRetryMs) return;
  if (latchAnnounced && torqueTries >= kTorqueRetryMax) return;
  lastTorqueTryMs = nowMs;
  ++torqueTries;
  torqueOffConfirmed = wheels.torqueOff();
  if (!latchAnnounced) {
    Serial.printf("[safe] LATCHED cause=%s (reset to re-arm)\n", bsup::name(sup.cause()));
    latchAnnounced = true;
  }
  if (torqueOffConfirmed) {
    Serial.println("[safe] torque off confirmed");
  } else if (torqueTries >= kTorqueRetryMax) {
    Serial.println("[safe] TORQUE OFF NOT CONFIRMED, gave up: REMOVE SERVO POWER");
  } else {
    Serial.println("[safe] TORQUE OFF NOT CONFIRMED, retrying");
  }
}

// Only 'x'/'X' is a command. There is NO re-arm: reset to re-arm.
static void pollSerial() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'x' || c == 'X') sup.onKillRequest(bsup::Cause::SerialKill);
  }
}

static void writeWheels(float cmd) {
  if (kWheelWritesEnabled) (void)wheels.writeRadS(cmd, cmd);
}

// ONE place: read IMU, step controller, write WheelPair, update LoopStats.
static void controlTick(uint32_t now) {
  const uint32_t period = now - lastTickUs;  // unsigned: wrap-safe, never negative or NaN
  const bool first = !haveTick || period == 0;
  lastTickUs = now;
  haveTick = true;
  // First tick returns the NOMINAL period (contract v1) and is not fed to loop_stats.
  const float dt = static_cast<float>(first ? kLoopUs : period) * 1e-6f;

  const bool imuOk = imu.read(sample, dt);
  const bsup::Tick t{first ? 0u : period, first, imuOk, imuOk ? sample.pitch_rad : 0.0f};
  const bsup::Action act = sup.onTick(t);

  float cmd = 0.0f;
  switch (act) {
    case bsup::Action::Arm:
      ctrl.reset();
      Serial.printf("[bal] %s ARMED (UNVERIFIED sign, wheels off ground)\n", kCtrlName);
      break;
    case bsup::Action::Drive: {
      const balance::ControlOutput out = ctrl.step(sample, dt);
      if (sup.onCommand(out.cmd_rad_s, out.fault)) {
        cmd = out.cmd_rad_s * kWheelCmdScale;
        writeWheels(cmd);
      }
      break;
    }
    case bsup::Action::Zero:
      writeWheels(0.0f);
      break;
    case bsup::Action::Idle:
      break;
  }
  sup.onWheels(wheels.alive());
  enforceLatch();
  if (!first) stats.add(period, kJitterBudgetUs);

  if (now - lastBalPrintUs >= kBalPrintUs) {
    lastBalPrintUs = now;
    Serial.printf("[bal] pitch=%.4f cmd=%.4f state=%s cause=%s imu=%d\n",
                  static_cast<double>(imuOk ? sample.pitch_rad : 0.0f), static_cast<double>(cmd),
                  bsup::name(sup.state()), bsup::name(sup.cause()), imuOk ? 1 : 0);
  }
  if (now - lastLoopPrintUs >= kLoopPrintUs) {
    lastLoopPrintUs = now;
    if (stats.count > 0) {
      Serial.printf("[loop] min=%lu mean=%lu max=%lu over=%lu n=%lu us\n",
                    static_cast<unsigned long>(stats.min_us), static_cast<unsigned long>(stats.mean_us()),
                    static_cast<unsigned long>(stats.max_us), static_cast<unsigned long>(stats.over_budget),
                    static_cast<unsigned long>(stats.count));
    }
  }
}

void setup() {
  Serial.begin(kUsbSerialBaud);
  const bool wheelsOk = startWheels();  // torque-off first, no delay before it
  delay(500);
  Serial.printf("[bal] boot %s (control loop, UNVERIFIED on hardware)\n", kCtrlName);
  reportWheels(wheelsOk);
  sup.onStartup(wheelsOk);
  enforceLatch();
  imuBegun = imu.begin(D4, D5, kI2cHz);
  Serial.println(imuBegun ? "[imu] ready" : "[imu] MPU6050 not found (check SDA=D4 SCL=D5, AD0)");
  if (imuBegun && tryCalibrate()) onCalibrated();
  lastTryMs = millis();
}

void loop() {
  pollSerial();
  enforceLatch();

  if (!imuBegun || !imu.calibrated()) {
    if (millis() - lastTryMs >= kRetryMs) {
      lastTryMs = millis();
      if (!imuBegun) {
        imuBegun = imu.begin(D4, D5, kI2cHz);
        Serial.println(imuBegun ? "[imu] ready" : "[imu] not found");
      }
      if (imuBegun && tryCalibrate()) onCalibrated();
    }
    return;
  }

  const uint32_t now = micros();
  if (static_cast<int32_t>(now - nextUs) < 0) return;
  nextUs += kLoopUs;
  if (static_cast<int32_t>(now - nextUs) >= 0) nextUs = now + kLoopUs;  // overran a whole period: resync, no burst
  controlTick(now);
}
