// ttl_servo: closed-loop Segway physics simulation for Wokwi, driving the
// simulated Dynamixel XL330 chip.
// SIM-ONLY, UNVERIFIED on hardware. The firmware simulates the Segway frame
// (lib/segway_sim) and balances it with a PD loop. The real MPU6050 gyro Z is
// a disturbance rate injected into the simulated tilt dynamics (move the
// Wokwi gyro Z slider to push the frame).
//
// Servo bus: Dynamixel XL330 chip host over Serial1, GPIO20 (RX) / GPIO21 (TX),
// 57600 baud, protocol 2.0, through the FE-URT-1 adapter (direction handled in
// hardware). Velocity mode. The goal is the simulated wheel's lagged velocity
// in raw units, capped on the host (segway::kWheelCapRaw), never a device read.
// 10 Hz telemetry also reads the present position (pos, dpos) for the sim check.
// 1 Hz [dxl] line reports transaction counts and the max transaction time (us).
// Fall: zero-velocity stop (3 tries, torque off if all fail), then hold 1 s and restart.
// 3 failed writes in a row: the firmware first stops the chip (velocity 0 with 3 tries,
// torque off if needed), then latches bus_down (no Dynamixel calls; reset the board to retry).
// The same stop-then-latch runs after a failed bring-up.
// -D TTL_SERVO_HORN_SELFTEST: after gyro calibration, 2 s forward + 2 s reverse at 90 deg/s, then 0.
// -D TTL_SERVO_FALL_TEST: low gains (kp=2 kd=1) so the frame falls within seconds.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <segway_sim.h>

// ===== Configuration =====
static constexpr uint8_t  I2C_SDA = 8;
static constexpr uint8_t  I2C_SCL = 9;
static constexpr uint8_t  MPU_ADDR = 0x68;
static constexpr uint8_t  DXL_ID = 1;
static constexpr float    DXL_PROTO = 2.0f;
static constexpr uint32_t DXL_BAUD = 57600;
static constexpr int      DXL_DIR_PIN = -1;   // FE-URT-1 handles direction in hardware
static constexpr int      DXL_RX = 20;        // D7
static constexpr int      DXL_TX = 21;        // D6
static constexpr uint32_t LOOP_PERIOD_MS = 20;         // 50 Hz main loop
static constexpr uint32_t PRINT_PERIOD_MS = 100;       // 10 Hz serial output
static constexpr float    SIM_INIT_THETA_RAD = 0.262f; // start leaning 15 deg
static constexpr float    GYRO_FILTER_TAU_MS = 50.0f;  // gyro Z low-pass
static constexpr int      GYRO_CAL_SAMPLES = 100;      // ~1 s at boot, sim at rest

// ===== Globals =====
Adafruit_MPU6050 mpu;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
segway::SegwaySim segway_sim(SIM_INIT_THETA_RAD);
uint32_t lastLoopMs = 0;
uint32_t lastPrintMs = 0;
float gyroZ_bias_rad_s = 0.0f;
float gyroZ_filtered_rad_s = 0.0f;
bool servo_ok = false;          // false: chip absent or setup failed, read-only sim
int32_t last_written_raw = 0;   // raw goal velocity most recently accepted by the chip
uint32_t write_fail = 0;
static uint32_t tx_ok, tx_fail, rd_ok, rd_fail, max_us;
static uint32_t lastBusPrintMs;
static int32_t last_pos = 0;
static bool have_last_pos = false;
static bool bus_down = false;        // latched: no Dynamixel calls until reset
static uint8_t consec_fail = 0;      // consecutive failed writes (reset on success)
static bool holding = false;         // true for 1 s after a fall
static uint32_t hold_until_ms = 0;

// ===== Helpers =====
static float ema_update(float prev, float raw, uint32_t dt_ms) {
  if (dt_ms == 0) return prev;
  float alpha = (float)dt_ms / (GYRO_FILTER_TAU_MS + (float)dt_ms);
  return prev * (1.0f - alpha) + raw * alpha;
}

// Measured dt in seconds, clamped so a stall cannot blow up the integrator.
static float clamp_dt_s(uint32_t dt_ms) {
  float dt = (float)dt_ms / 1000.0f;
  if (dt < 0.005f) dt = 0.005f;
  if (dt > 0.1f) dt = 0.1f;
  return dt;
}

// Goal velocity 0, up to 3 attempts. Each attempt is timed and counted.
static bool write_vel_zero_with_retries() {
  for (uint8_t i = 0; i < 3; ++i) {
    uint32_t t0 = micros();
    bool ok = dxl.setGoalVelocity(DXL_ID, 0.0f, UNIT_RAW);
    uint32_t el = micros() - t0;
    if (el > max_us) max_us = el;
    if (ok) {
      ++tx_ok;
      consec_fail = 0;
      return true;
    }
    ++tx_fail;
    ++consec_fail;
  }
  return false;
}

// force_torque_off: also run the torque-off sequence after a successful zero
// write (the chip may be in position mode); returns true only if torque-off
// was confirmed in that case.
static bool stop_chip_best_effort(const char* why, bool force_torque_off = false) {
  bool zok = write_vel_zero_with_retries();
  Serial.printf("[dxl] stop vel=0 ok=%d\n", zok ? 1 : 0);
  if (zok) {
    last_written_raw = 0;
    if (!force_torque_off) return true;
  }
  bool off = false;
  // Torque-off transactions are not counted in tx_ok/tx_fail/max_us (stats cover goal-velocity writes and position reads only).
  for (uint8_t i = 0; i < 3 && !off; ++i) off = dxl.torqueOff(DXL_ID);
  Serial.printf("[safe] torque off %s%s\n", why,
                off ? "" : " (TORQUE OFF NOT CONFIRMED)");
  return force_torque_off ? off : false;
}

// Serial1 + protocol setup for the servo bus. Called once, before any bus use.
static void dxl_bus_begin() {
  dxl.begin(DXL_BAUD);
  Serial1.begin(DXL_BAUD, SERIAL_8N1, DXL_RX, DXL_TX);
  dxl.setPortProtocolVersion(DXL_PROTO);
}

// Latch: no Dynamixel calls until reset. Physics and telemetry keep running.
static void latch_bus_down() {
  servo_ok = false;
  if (bus_down) return;
  bus_down = true;
  Serial.printf("[dxl] tx_ok=%lu tx_fail=%lu rd_ok=%lu rd_fail=%lu max_us=%lu\n",
                (unsigned long)tx_ok, (unsigned long)tx_fail,
                (unsigned long)rd_ok, (unsigned long)rd_fail,
                (unsigned long)max_us);
  Serial.println("[safe] bus down (reset the board to retry)");
}

#ifdef TTL_SERVO_HORN_SELFTEST
// Velocity self-test: 2 s forward, 2 s reverse at 90 deg/s, then 0. Prints the
// chip's present position every 250 ms. Blocks boot; needs a live servo.
static bool selftest_read_pos(int32_t* pos) {
  *pos = dxl.getPresentPosition(DXL_ID);
  return dxl.getLastLibErrCode() == DXL_LIB_OK;
}

static void horn_selftest() {
  if (!servo_ok) {
    Serial.println("[ttl_servo] selftest skipped (no servo)");
    return;
  }
  const int32_t raw = segway::wheel_cmd_to_raw(90.0f / segway::kRadToDeg);
  const int32_t cmds[3] = {raw, -raw, 0};
  const int steps[3] = {8, 8, 1};
  int32_t prev = 0;
  bool have_prev = selftest_read_pos(&prev);
  for (int ph = 0; ph < 3; ++ph) {
    if (!dxl.setGoalVelocity(DXL_ID, (float)cmds[ph], UNIT_RAW)) {
      ++tx_fail;
      Serial.printf("[ttl_servo] selftest write failed cmd=%ld\n", (long)cmds[ph]);
      (void)stop_chip_best_effort("selftest write failed");
      latch_bus_down();
      return;
    }
    ++tx_ok;
    last_written_raw = cmds[ph];
    for (int i = 0; i < steps[ph]; ++i) {
      delay(250);
      int32_t pos = 0;
      if (selftest_read_pos(&pos) && have_prev) {
        Serial.printf("[ttl_servo] selftest cmd=%ld pos=%ld dpos=%ld\n", (long)cmds[ph],
                      (long)pos, (long)(((pos - prev + 6144) % 4096) - 2048));
        prev = pos;
      } else {
        Serial.printf("[ttl_servo] selftest cmd=%ld pos=ERR\n", (long)cmds[ph]);
        have_prev = false;
      }
    }
  }
  have_last_pos = false;  // telemetry dpos restarts from its first read
}
#endif

// ===== Arduino entry points =====
void setup() {
  Serial.begin(115200);
  delay(1000); // Boot without blocking on USB host

  dxl_bus_begin();
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("\n[ttl_servo] Segway physics sim (Wokwi, SIM-ONLY, UNVERIFIED)");

  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("[ttl_servo] Failed to find MPU6050 chip");
    // Ping first: Dynamixel2Arduino resolves registers through the model number a ping fills in,
    // so a write without a prior ping fails even when the servo is present.
    (void)dxl.ping(DXL_ID);
    (void)stop_chip_best_effort("no IMU", true);
    while (1) { delay(10); }
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Servo bring-up before the (blocking) calibration; the bus was begun above.
  bool ping_ok = dxl.ping(DXL_ID);
  Serial.printf("[dxl] ping %s\n", ping_ok ? "ok" : "FAILED");
  if (!ping_ok) {
    Serial.println("[dxl] servo not responding, read-only");
  } else {
    // The chip boots in position mode: OP_VELOCITY must be set before any
    // velocity write. Each step is skipped (reported 0) once one fails.
    bool w1 = dxl.torqueOff(DXL_ID);
    Serial.printf("[dxl] torque_off=%d\n", w1);
    bool w2 = w1 && dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
    Serial.printf("[dxl] mode_velocity=%d\n", w2);
    bool w3 = w2 && dxl.setGoalVelocity(DXL_ID, 0.0f, UNIT_RAW);
    Serial.printf("[dxl] goal0=%d\n", w3);
    bool w4 = w3 && dxl.torqueOn(DXL_ID);
    Serial.printf("[dxl] torque_on=%d\n", w4);
    servo_ok = w4;
    if (!servo_ok) {
      (void)dxl.torqueOff(DXL_ID);  // best effort, torqueOn may have landed
      Serial.println("[dxl] config failed, read-only");
    }
  }
  if (!servo_ok) {
    // Blocks boot while the servo is absent. Worst case about 7 timed-out
    // transactions (ping, 3 zero writes, 3 torque-off), roughly 140 ms in the sim;
    // up to 12 (about 240 ms) if ping answers and the config fails at its last step
    // (ping + 4 config steps + the config-fail torque-off + 3 zero writes + 3 torque-off).
    (void)stop_chip_best_effort("bring-up failed", true);
    latch_bus_down();
  }

  // Gyro Z bias: average with the Wokwi gyro slider at rest.
  float sum = 0.0f;
  int good = 0;
  for (int i = 0; i < GYRO_CAL_SAMPLES; ++i) {
    sensors_event_t a, g, t;
    if (mpu.getEvent(&a, &g, &t)) {
      sum += g.gyro.z;
      ++good;
    }
    delay(10);
  }
  if (good < GYRO_CAL_SAMPLES / 2) {
    Serial.println("[ttl_servo] gyro calibration failed (IMU reads), halting");
    while (1) { delay(10); }
  }
  gyroZ_bias_rad_s = sum / (float)good;
  Serial.printf("[ttl_servo] gyro Z bias=%.4f rad/s (keep the slider at 0 during boot)\n",
                gyroZ_bias_rad_s);

#ifdef TTL_SERVO_HORN_SELFTEST
  horn_selftest();
#endif
  segway_sim.reset(SIM_INIT_THETA_RAD);
#ifdef TTL_SERVO_FALL_TEST
  segway_sim.params.kp = 2.0f;
  segway_sim.params.kd = 1.0f;
  Serial.println("[ttl_servo] FALL TEST: kp=2 kd=1 (expect FELL within ~6 s)");
#endif
  Serial.println("[ttl_servo] Sim starts at 15 deg; PD should settle to 0. Move gyro Z to push.\n");

  lastLoopMs = millis();
  lastPrintMs = lastLoopMs;
  lastBusPrintMs = lastLoopMs;
}

void loop() {
  uint32_t now_ms = millis();
  uint32_t dt_ms = now_ms - lastLoopMs;
  if (dt_ms < LOOP_PERIOD_MS) {
    delay(1);
    return;
  }
  lastLoopMs = now_ms;

  // 1. Real gyro Z (bias removed, filtered) is the disturbance rate.
  sensors_event_t accel, gyro, temp;
  // On a failed read keep the previous filtered value rather than stale data.
  if (mpu.getEvent(&accel, &gyro, &temp)) {
    float gz = gyro.gyro.z - gyroZ_bias_rad_s;
    gyroZ_filtered_rad_s = ema_update(gyroZ_filtered_rad_s, gz, dt_ms);
  } else {
    Serial.println("[ttl_servo] IMU read failed, holding last gyro Z");
  }

  // 2. Hold after a fall: physics frozen, wheel stays stopped, then restart.
  if (holding) {
    if ((int32_t)(now_ms - hold_until_ms) >= 0) {
      segway_sim.reset(SIM_INIT_THETA_RAD);
#ifdef TTL_SERVO_FALL_TEST
      segway_sim.params.kp = 2.0f;
      segway_sim.params.kd = 1.0f;
#endif
      holding = false;
      Serial.println("[ttl_servo] restart");
    }
  } else {
    // 3. PD -> physics, same code the native tests exercise. The returned PD
    // command is ignored: the chip gets the plant's lagged wheel velocity.
    float dt_s = clamp_dt_s(dt_ms);
    (void)segway_sim.tick_cmd(gyroZ_filtered_rad_s, dt_s);

    if (segway_sim.has_fallen) {
      // 4. Fall first, so a fall tick never writes the lagged velocity.
      Serial.printf("[ttl_servo] FELL theta=%.3f\n", segway_sim.theta_rad);
      if (servo_ok) {
        if (!stop_chip_best_effort("after fall")) latch_bus_down();
      }
      hold_until_ms = millis() + 1000;
      holding = true;
    } else if (servo_ok) {
      // Goal velocity to the chip. Only on change, only if the chip is live.
      int32_t raw = segway::wheel_cmd_to_raw(segway_sim.wheel_vel_rad_s);
      if (raw != last_written_raw) {
        uint32_t t0 = micros();
        bool wok = dxl.setGoalVelocity(DXL_ID, (float)raw, UNIT_RAW);
        uint32_t el = micros() - t0;
        if (el > max_us) max_us = el;
        if (wok) {
          ++tx_ok;
          consec_fail = 0;
          last_written_raw = raw;
        } else {
          ++tx_fail;
          ++write_fail;
          ++consec_fail;
          if (write_fail == 1 || write_fail % 50 == 0)
            Serial.printf("[dxl] write failed raw=%ld (n=%lu)\n",
                          (long)raw, (unsigned long)write_fail);
          if (consec_fail >= 3) {
            (void)stop_chip_best_effort("before bus latch");
            latch_bus_down();
          }
        }
      }
    }
  }

  // 5. Telemetry (10 Hz)
  if (now_ms - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now_ms;
    char pos_s[12] = "NA";
    char dpos_s[12] = "n/a";
    if (servo_ok) {
      uint32_t t0 = micros();
      int32_t pos = dxl.getPresentPosition(DXL_ID);
      bool rd = (dxl.getLastLibErrCode() == DXL_LIB_OK);
      uint32_t el = micros() - t0;
      if (el > max_us) max_us = el;
      if (rd) {
        ++rd_ok;
        snprintf(pos_s, sizeof(pos_s), "%ld", (long)pos);
        if (have_last_pos) {
          int32_t dpos = ((pos - last_pos + 6144) % 4096) - 2048;
          snprintf(dpos_s, sizeof(dpos_s), "%ld", (long)dpos);
        }
        last_pos = pos;
        have_last_pos = true;
      } else {
        ++rd_fail;
        snprintf(pos_s, sizeof(pos_s), "ERR");
        have_last_pos = false;
      }
    }
    // cmd = last value CONFIRMED written to the chip; stale (non-zero) after a failed stop.
    // With bus=1, cmd=0 before the first loop write is the chip's real goal (bring-up confirmed goal 0);
    // with bus=0 cmd only holds the last confirmed value and says nothing about the chip.
    Serial.printf("[ttl_servo] sim theta=%.3f rate=%.3f wheel=%.2f cmd=%ld pos=%s dpos=%s gyroZ=%.3f bus=%d\n",
                  segway_sim.theta_rad, segway_sim.theta_dot_rad_s,
                  segway_sim.wheel_vel_rad_s, (long)last_written_raw,
                  pos_s, dpos_s, gyroZ_filtered_rad_s, servo_ok ? 1 : 0);
  }

  if (servo_ok && now_ms - lastBusPrintMs >= 1000) {
    Serial.printf("[dxl] tx_ok=%lu tx_fail=%lu rd_ok=%lu rd_fail=%lu max_us=%lu\n",
                  (unsigned long)tx_ok, (unsigned long)tx_fail,
                  (unsigned long)rd_ok, (unsigned long)rd_fail,
                  (unsigned long)max_us);
    max_us = 0;
    lastBusPrintMs = now_ms;
  }
}
