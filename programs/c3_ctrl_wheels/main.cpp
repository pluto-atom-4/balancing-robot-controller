// c3_ctrl_wheels: sim-only twin of ttl_servo, ID 1 left (W[0]), ID 2 right (W[1], unconfirmed). UNVERIFIED, not hardware-validated.
// A relay to Webots, not a closed loop. The wheel sign is applied in write_goal_raw only. Sim IMU I2C on GPIO8/9 is a sim-only wiring choice.
// Log tags: [ttl_servo], [dxl], [safe], [ctrl_wheel].

// WHL2 raws are the values last CONFIRMED on each chip; unknown and confirmed 0 both print 0, state 2 prints 0/0. State 0 does not mean the writes succeeded.
// State 2 (bus_down latched) is the only latched (terminal) state (1 is hold after a fall); the boot-time halts print no WHL2 line.
// A tick fails if either write fails; 3 failed ticks in a row force torque off on both ids, then latch (reset to retry).

// -D C3_CTRL_WHEELS_TEST=1..4 (default OFF): SIM-ONLY, UNVERIFIED failure injection, not hardware-validated; remove before uploading to a board (G10, #309). Results: #304.
// A run counts as a preset run only if the log from reset shows the `[ttl_servo] TEST INJECT ON preset=N after=M` banner; a log without it is not proof of default firmware.

// Presets from test tick M (live non-holding 20 ms passes): 1 both writes fail; 2 ID 2 writes fail; 3 ID 2 for [M, M+2);
// 4 ID 2 read verdict for [M, M+50) ticks (about 10 reads at 10 Hz, not 50; active through a hold).
// Forced write failures bypass write-on-change (attempted even when the value equals last_raw) and skip setGoalVelocity; counters include injected events.

// Presets 3 and 4 do not latch; no stop path is injected; failed zero writes and torque-off failures are not covered.
#if defined(C3_CTRL_WHEELS_TEST) && ((C3_CTRL_WHEELS_TEST < 0) || (C3_CTRL_WHEELS_TEST > 4))
#error "C3_CTRL_WHEELS_TEST must be 0 (off) or 1..4"
#endif
#if defined(C3_CTRL_WHEELS_TEST_AFTER) && (C3_CTRL_WHEELS_TEST_AFTER < 0)
#error "C3_CTRL_WHEELS_TEST_AFTER must be 0 or more"
#endif
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Dynamixel2Arduino.h>
#include <segway_sim.h>
#include <wheels_frame.h>

static constexpr uint8_t  I2C_SDA = 8;
static constexpr uint8_t  I2C_SCL = 9;
static constexpr uint8_t  MPU_ADDR = 0x68;
static constexpr float    DXL_PROTO = 2.0f;
static constexpr uint32_t DXL_BAUD = 57600;
static constexpr int      DXL_DIR_PIN = -1;   // FE-URT-1 handles direction in hardware
static constexpr int      DXL_RX = 20;
static constexpr int      DXL_TX = 21;
static constexpr uint32_t LOOP_PERIOD_MS = 20;
static constexpr uint32_t PRINT_PERIOD_MS = 100;
static constexpr float    SIM_INIT_THETA_RAD = 0.262f;
static constexpr float    GYRO_FILTER_TAU_MS = 50.0f;
static constexpr int      GYRO_CAL_SAMPLES = 100;
// SIM-ONLY, UNVERIFIED. The only place wheel signs are defined; IDs must match servoId in diagram.json.
// A chip with no servoId answers as ID 1, so servo2 must carry servoId "2".
static constexpr uint8_t SERVO_ID_LEFT    = 1;
static constexpr uint8_t SERVO_ID_RIGHT   = 2;   // unconfirmed
static constexpr int8_t  WHEEL_SIGN_LEFT  = 1;   // unconfirmed
static constexpr int8_t  WHEEL_SIGN_RIGHT = -1;  // unconfirmed mirror

Adafruit_MPU6050 mpu;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
uint32_t lastLoopMs = 0;
uint32_t lastPrintMs = 0;
float gyroZ_bias_rad_s = 0.0f;
float gyroZ_filtered_rad_s = 0.0f;
bool servo_ok = false;
uint32_t write_fail = 0;
static uint32_t tx_ok, tx_fail, max_us;
static uint32_t lastBusPrintMs;
static bool bus_down = false;
static uint8_t tick_fail_streak = 0;
static bool holding = false;
static uint32_t hold_until_ms = 0;
// WHL2 buffer: 96 bytes is a sizing choice, not measured; format returns 0 if the line does not fit (counted in dropped).
// whl_seq wraps after 65536 ticks (about 22 min at 50 Hz) and t_us after about 71.6 min; the host must handle both.
static uint16_t whl_seq = 0;
static uint32_t whl_sent = 0, whl_dropped = 0, whl_max_emit_us = 0, lastWhlPrintMs = 0;
static char whl_buf[96];
struct WheelCtx {
  uint8_t  id;
  int8_t   sign;
  segway::SegwaySim sim;
  uint8_t  consec_fail;
  int32_t  last_raw;       // INT32_MIN = unknown, so the next write always goes out
  int32_t  last_pos;
  bool     have_last_pos;
  uint32_t rd_ok, rd_fail;
  WheelCtx(uint8_t id_, int8_t sign_)
    : id(id_), sign(sign_), sim(SIM_INIT_THETA_RAD), consec_fail(0),
      last_raw(INT32_MIN), last_pos(0), have_last_pos(false), rd_ok(0), rd_fail(0) {}
};
static WheelCtx W[2] = { WheelCtx(SERVO_ID_LEFT, WHEEL_SIGN_LEFT),
                         WheelCtx(SERVO_ID_RIGHT, WHEEL_SIGN_RIGHT) };

#if defined(C3_CTRL_WHEELS_TEST) && (C3_CTRL_WHEELS_TEST > 0)
// SIM-ONLY failure injection; test_tick is 1-based.
#ifndef C3_CTRL_WHEELS_TEST_AFTER
#define C3_CTRL_WHEELS_TEST_AFTER 100
#endif
static uint32_t test_tick = 0;
static bool test_fail_write(const WheelCtx& w) {
#if C3_CTRL_WHEELS_TEST == 1
  const uint32_t m = (uint32_t)C3_CTRL_WHEELS_TEST_AFTER;
  (void)w;
  return test_tick >= m;
#elif C3_CTRL_WHEELS_TEST == 2
  const uint32_t m = (uint32_t)C3_CTRL_WHEELS_TEST_AFTER;
  return w.id == SERVO_ID_RIGHT && test_tick >= m;
#elif C3_CTRL_WHEELS_TEST == 3
  const uint32_t m = (uint32_t)C3_CTRL_WHEELS_TEST_AFTER;
  return w.id == SERVO_ID_RIGHT && test_tick >= m && test_tick < m + 2;
#else
  (void)w;
  return false;
#endif
}
static bool test_fail_read(const WheelCtx& w) {
#if C3_CTRL_WHEELS_TEST == 4
  const uint32_t m = (uint32_t)C3_CTRL_WHEELS_TEST_AFTER;
  return w.id == SERVO_ID_RIGHT && test_tick >= m && test_tick < m + 50;
#else
  (void)w;
  return false;
#endif
}
#endif

static float ema_update(float prev, float raw, uint32_t dt_ms) {
  if (dt_ms == 0) return prev;
  float alpha = (float)dt_ms / (GYRO_FILTER_TAU_MS + (float)dt_ms);
  return prev * (1.0f - alpha) + raw * alpha;
}

// Clamped so a stall cannot blow up the integrator.
static float clamp_dt_s(uint32_t dt_ms) {
  float dt = (float)dt_ms / 1000.0f;
  if (dt < 0.005f) dt = 0.005f;
  if (dt > 0.1f) dt = 0.1f;
  return dt;
}


static int32_t whl2_raw(const WheelCtx& w) {
  return (w.last_raw == INT32_MIN) ? 0 : w.last_raw;
}

// Guarded by availableForWrite(); whether Serial.write blocks on the C3 USB CDC is unverified. Other prints are unguarded.
static void whl_emit(float model_rad_s, int32_t rawL, int32_t rawR, uint8_t state) {
  uint32_t t0 = micros();
  size_t n = wheels_frame::format(whl_buf, sizeof whl_buf, whl_seq, (uint32_t)micros(),
                                  model_rad_s, rawL, rawR, state);
  whl_seq++;
  bool written = false;
  if (n != 0) {
    int avail = Serial.availableForWrite();
    if (avail > 0 && (size_t)avail >= n) {
      Serial.write((const uint8_t*)whl_buf, n);
      written = true;
    }
  }
  if (written) whl_sent++; else whl_dropped++;
  uint32_t el = micros() - t0;
  if (el > whl_max_emit_us) whl_max_emit_us = el;
}


// Not gated by bus_down: part of the kill/torque-off path, which must still try after a latch.
static bool write_vel_zero_ctx(WheelCtx& w) {
  for (uint8_t i = 0; i < 3; ++i) {
    uint32_t t0 = micros();
    bool ok = dxl.setGoalVelocity(w.id, 0.0f, UNIT_RAW);
    uint32_t el = micros() - t0;
    if (el > max_us) max_us = el;
    if (ok) {
      ++tx_ok;
      w.consec_fail = 0;
      w.last_raw = 0;
      return true;
    }
    ++tx_fail;
    if (w.consec_fail < 255) ++w.consec_fail;
  }
  w.last_raw = INT32_MIN;
  return false;
}

// Do not multiply by .sign elsewhere; raw is already capped at +-segway::kWheelCapRaw, do not re-cap.
// INT32_MIN = unknown: a timed-out write may have landed, so the next write always goes out.
static bool write_goal_raw(WheelCtx& w, int32_t raw) {
  if (bus_down) return false;
  int32_t wire = (int32_t)w.sign * raw;
#if defined(C3_CTRL_WHEELS_TEST) && (C3_CTRL_WHEELS_TEST > 0)
  // SIM-ONLY failure injection: a forced failure skips the bus call and counts as a failed write.
  bool force_fail = test_fail_write(w);
  if (!force_fail && wire == w.last_raw) return true;
  uint32_t t0 = micros();
  bool ok = force_fail ? false : dxl.setGoalVelocity(w.id, (float)wire, UNIT_RAW);
#else
  if (wire == w.last_raw) return true;
  uint32_t t0 = micros();
  bool ok = dxl.setGoalVelocity(w.id, (float)wire, UNIT_RAW);
#endif
  uint32_t el = micros() - t0;
  if (el > max_us) max_us = el;
  if (ok) {
    ++tx_ok;
    w.consec_fail = 0;
    w.last_raw = wire;
    return true;
  }
  ++tx_fail;
  if (w.consec_fail < 255) ++w.consec_fail;
  w.last_raw = INT32_MIN;
  return false;
}

// Stops BOTH wheels, always attempting both. Not gated by bus_down: kill/torque-off path, must still try after a latch.
// Force returns the torque-off result only (the stronger stop). Dead bus: up to 12 transactions before the latch (unmeasured, sim).
// last_raw = INT32_MIN is set once per wheel after its up to 3 torqueOff attempts, so the next write goes out even after torque is re-enabled.
static bool stop_wheels_best_effort(const char* why, bool force_torque_off = false) {
  bool zok_all = true;
  for (uint8_t k = 0; k < 2; ++k) {
    bool zok = write_vel_zero_ctx(W[k]);
    Serial.printf("[dxl] stop id=%lu vel=0 ok=%d\n", (unsigned long)W[k].id, zok ? 1 : 0);
    if (!zok) zok_all = false;
  }
  if (zok_all && !force_torque_off) return true;
  bool off_all = true;
  for (uint8_t k = 0; k < 2; ++k) {
    bool off = false;
    for (uint8_t i = 0; i < 3 && !off; ++i) off = dxl.torqueOff(W[k].id);
    Serial.printf("[safe] torque off %s id=%lu%s\n", why, (unsigned long)W[k].id,
                  off ? "" : " (TORQUE OFF NOT CONFIRMED)");
    if (!off) off_all = false;
    W[k].last_raw = INT32_MIN;
  }
  return force_torque_off ? off_all : false;
}

static void dxl_bus_begin() {
  dxl.begin(DXL_BAUD);
  Serial1.begin(DXL_BAUD, SERIAL_8N1, DXL_RX, DXL_TX);
  dxl.setPortProtocolVersion(DXL_PROTO);
}

// Latch: write_goal_raw refuses and the loop's servo_ok paths skip chip I/O; stop_wheels_best_effort is not gated.
static void latch_bus_down() {
  servo_ok = false;
  if (bus_down) return;
  bus_down = true;
  // Keep this [dxl] format string in sync with the 1 Hz [dxl] line in loop().
  Serial.printf("[dxl] tx_ok=%lu tx_fail=%lu rd_ok1=%lu rd_fail1=%lu rd_ok2=%lu rd_fail2=%lu max_us=%lu\n",
                (unsigned long)tx_ok, (unsigned long)tx_fail,
                (unsigned long)W[0].rd_ok, (unsigned long)W[0].rd_fail,
                (unsigned long)W[1].rd_ok, (unsigned long)W[1].rd_fail,
                (unsigned long)max_us);
  Serial.println("[safe] bus down (reset the board to retry)");
}

// Ping first: Dynamixel2Arduino resolves registers through the model number a ping fills in (library behaviour, unchecked).
static bool bring_up_wheel(WheelCtx& w) {
  bool ok = dxl.ping(w.id);
  Serial.printf("[dxl] ping id=%lu %s\n", (unsigned long)w.id, ok ? "ok" : "FAILED");
  if (ok) {
    ok = dxl.torqueOff(w.id);
    Serial.printf("[dxl] torque_off id=%lu ok=%d\n", (unsigned long)w.id, ok ? 1 : 0);
  }
  if (ok) {
    ok = dxl.setOperatingMode(w.id, OP_VELOCITY);
    Serial.printf("[dxl] mode_velocity id=%lu ok=%d\n", (unsigned long)w.id, ok ? 1 : 0);
  }
  if (ok) {
    ok = write_vel_zero_ctx(w);
    Serial.printf("[dxl] goal0 id=%lu ok=%d\n", (unsigned long)w.id, ok ? 1 : 0);
  }
  if (ok) {
    ok = dxl.torqueOn(w.id);
    Serial.printf("[dxl] torque_on id=%lu ok=%d\n", (unsigned long)w.id, ok ? 1 : 0);
  }
  if (!ok) {
    (void)dxl.torqueOff(w.id);  // best effort, a failed step may have landed
    w.last_raw = INT32_MIN;
    Serial.printf("[dxl] bring-up id=%lu failed\n", (unsigned long)w.id);
  }
  return ok;
}

// The FALL_TEST gains are applied at boot and on every restart.
static void reset_sims() {
  for (uint8_t k = 0; k < 2; ++k) {
    W[k].sim.reset(SIM_INIT_THETA_RAD);
#ifdef TTL_SERVO_FALL_TEST
    W[k].sim.params.kp = 2.0f;
    W[k].sim.params.kd = 1.0f;
#endif
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  dxl_bus_begin();
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("\n[ttl_servo] Segway physics sim (Wokwi, SIM-ONLY, UNVERIFIED)");
#if defined(C3_CTRL_WHEELS_TEST) && (C3_CTRL_WHEELS_TEST > 0)
  Serial.printf("[ttl_servo] TEST INJECT ON preset=%d after=%d (SIM-ONLY, remove before uploading to a physical board)\n",
                (int)C3_CTRL_WHEELS_TEST, (int)C3_CTRL_WHEELS_TEST_AFTER);
#endif

  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("[ttl_servo] Failed to find MPU6050 chip");
    // A write without a prior ping fails even when the servo is present (library behaviour, unchecked).
    for (uint8_t k = 0; k < 2; ++k) (void)dxl.ping(W[k].id);
    (void)stop_wheels_best_effort("no IMU", true);
    while (1) { delay(10); }
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  bool all_ok = true;
  for (uint8_t k = 0; k < 2; ++k) {
    if (!bring_up_wheel(W[k])) {
      all_ok = false;
      break;
    }
  }
  servo_ok = all_ok;
  if (!servo_ok) {
    Serial.println("[dxl] bring-up failed, bus down, no Dynamixel calls");
    // Boot-block worst case about 24 transactions, roughly 480 ms at an assumed 20 ms per timeout (unmeasured, sim).
    (void)stop_wheels_best_effort("bring-up failed", true);
    latch_bus_down();
  }

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
    if (!bus_down) (void)stop_wheels_best_effort("gyro cal failed", true);
    while (1) { delay(10); }
  }
  gyroZ_bias_rad_s = sum / (float)good;
  Serial.printf("[ttl_servo] gyro Z bias=%.4f rad/s (keep the slider at 0 during boot)\n",
                gyroZ_bias_rad_s);

  reset_sims();
#ifdef TTL_SERVO_FALL_TEST
  Serial.println("[ttl_servo] FALL TEST: kp=2 kd=1 (expect FELL within ~6 s)");
#endif
  Serial.println("[ttl_servo] Sim starts at 15 deg; PD should settle to 0. Move gyro Z to push.\n");

  lastLoopMs = millis();
  lastPrintMs = lastLoopMs;
  lastBusPrintMs = lastLoopMs;
  lastWhlPrintMs = lastLoopMs;
}

// Two plants per loop: soft-float cost unmeasured (sim).
void loop() {
  uint32_t now_ms = millis();
  uint32_t dt_ms = now_ms - lastLoopMs;
  if (dt_ms < LOOP_PERIOD_MS) {
    delay(1);
    return;
  }
  lastLoopMs = now_ms;

  sensors_event_t accel, gyro, temp;
  if (mpu.getEvent(&accel, &gyro, &temp)) {
    float gz = gyro.gyro.z - gyroZ_bias_rad_s;
    gyroZ_filtered_rad_s = ema_update(gyroZ_filtered_rad_s, gz, dt_ms);
  } else {
    Serial.println("[ttl_servo] IMU read failed, holding last gyro Z");
  }

  if (holding) {
    if ((int32_t)(now_ms - hold_until_ms) >= 0) {
      reset_sims();
      holding = false;
      tick_fail_streak = 0;
      Serial.println("[ttl_servo] restart");
    }
  } else {
    // The returned PD commands are ignored: each chip gets its plant's lagged wheel velocity.
    float dt_s = clamp_dt_s(dt_ms);
    for (uint8_t k = 0; k < 2; ++k) (void)W[k].sim.tick_cmd(gyroZ_filtered_rad_s, dt_s);

    if (W[0].sim.has_fallen || W[1].sim.has_fallen) {
      // Fall first, so a fall tick never writes the lagged velocity. Either plant falling stops both.
      uint8_t f = W[0].sim.has_fallen ? 0 : 1;
      Serial.printf("[ttl_servo] FELL id=%lu theta=%.3f\n",
                    (unsigned long)W[f].id, W[f].sim.theta_rad);
      // Non-force stop: torque off only if a zero write failed; a stop where both zero writes succeed leaves torque ON at zero velocity.
      // After a latch no stop is sent; state stays 2.
      if (servo_ok) {
        if (!stop_wheels_best_effort("after fall")) latch_bus_down();
      }
      hold_until_ms = millis() + 1000;
      holding = true;
    } else if (servo_ok) {
      // A failed wheel keeps last_raw = INT32_MIN so it retries every tick; a tick with no bus call resets the streak.
      int32_t raw0 = segway::wheel_cmd_to_raw(W[0].sim.wheel_vel_rad_s);
      int32_t raw1 = segway::wheel_cmd_to_raw(W[1].sim.wheel_vel_rad_s);
#if defined(C3_CTRL_WHEELS_TEST) && (C3_CTRL_WHEELS_TEST > 0)
      ++test_tick;
#endif
      bool ok0 = write_goal_raw(W[0], raw0);
      bool ok1 = write_goal_raw(W[1], raw1);
      if (ok0 && ok1) {
        tick_fail_streak = 0;
      } else {
        ++write_fail;
        // raw is before the wheel sign (wire = sign * raw); write_fail counts failed ticks, so both wheels failing prints two lines with the same n.
        if (write_fail == 1 || write_fail % 50 == 0) {
          if (!ok0)
            Serial.printf("[dxl] write failed id=%lu raw=%ld (n=%lu)\n",
                          (unsigned long)W[0].id, (long)raw0, (unsigned long)write_fail);
          if (!ok1)
            Serial.printf("[dxl] write failed id=%lu raw=%ld (n=%lu)\n",
                          (unsigned long)W[1].id, (long)raw1, (unsigned long)write_fail);
        }
        ++tick_fail_streak;
        if (tick_fail_streak >= 3) {
          (void)stop_wheels_best_effort("before bus latch", true);
          latch_bus_down();
        }
      }
    }
  }

  // model is the left wheel's lagged model value, not a measurement; 0 while holding.
  const float whl_model = holding ? 0.0f : W[0].sim.wheel_vel_rad_s;
  const uint8_t whl_state = bus_down ? 2 : (holding ? 1 : 0);
  const int32_t whl_rawL = (whl_state == 2) ? 0 : whl2_raw(W[0]);
  const int32_t whl_rawR = (whl_state == 2) ? 0 : whl2_raw(W[1]);
  whl_emit(whl_model, whl_rawL, whl_rawR, whl_state);

  if (now_ms - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now_ms;
    char pos_s[2][12] = { "NA", "NA" };
    char dpos_s[2][12] = { "n/a", "n/a" };
    char cmd_s[12] = "NA";
    if (servo_ok) {
      // [dxl] max_us excludes untimed calls (torque-off, ping, setOperatingMode, torqueOn, no-IMU ping), so it is not the worst call on the bus.
      // Reads never count toward write_fail or tick_fail_streak and never latch; a blocking read on a dead bus can stall the tick (unmeasured).
      for (uint8_t k = 0; k < 2; ++k) {
        uint32_t t0 = micros();
        int32_t pos = dxl.getPresentPosition(W[k].id);
        bool rd = (dxl.getLastLibErrCode() == DXL_LIB_OK);
#if defined(C3_CTRL_WHEELS_TEST) && (C3_CTRL_WHEELS_TEST > 0)
        // SIM-ONLY failure injection: the real read still went out; only the verdict is overridden.
        rd = rd && !test_fail_read(W[k]);
#endif
        uint32_t el = micros() - t0;
        if (el > max_us) max_us = el;
        if (rd) {
          ++W[k].rd_ok;
          snprintf(pos_s[k], sizeof(pos_s[k]), "%ld", (long)pos);
          if (W[k].have_last_pos) {
            int32_t dpos = ((pos - W[k].last_pos + 6144) % 4096) - 2048;
            snprintf(dpos_s[k], sizeof(dpos_s[k]), "%ld", (long)dpos);
          }
          W[k].last_pos = pos;
          W[k].have_last_pos = true;
        } else {
          ++W[k].rd_fail;
          snprintf(pos_s[k], sizeof(pos_s[k]), "ERR");
          W[k].have_last_pos = false;
        }
      }
    }
    // The wheel is shown as 0 during the post-fall hold whether or not the stop was confirmed.
    if (W[0].last_raw != INT32_MIN)
      snprintf(cmd_s, sizeof(cmd_s), "%ld", (long)W[0].last_raw);
    float wheel_shown = holding ? 0.0f : W[0].sim.wheel_vel_rad_s;
    Serial.printf("[ttl_servo] sim theta=%.3f rate=%.3f wheel=%.2f cmd=%s pos1=%s dpos1=%s pos2=%s dpos2=%s gyroZ=%.3f bus=%d\n",
                  W[0].sim.theta_rad, W[0].sim.theta_dot_rad_s,
                  wheel_shown, cmd_s,
                  pos_s[0], dpos_s[0], pos_s[1], dpos_s[1], gyroZ_filtered_rad_s, servo_ok ? 1 : 0);
  }

  // 80 bytes is a sizing choice, not measured.
  if (now_ms - lastWhlPrintMs >= 1000 && Serial.availableForWrite() >= 80) {
    Serial.printf("[ctrl_wheel] sent=%lu dropped=%lu max_emit_us=%lu\n",
                  (unsigned long)whl_sent, (unsigned long)whl_dropped,
                  (unsigned long)whl_max_emit_us);
    whl_max_emit_us = 0;
    lastWhlPrintMs = now_ms;
  }
  if (servo_ok && now_ms - lastBusPrintMs >= 1000) {
    // Keep this [dxl] format string in sync with the [dxl] line in latch_bus_down().
    Serial.printf("[dxl] tx_ok=%lu tx_fail=%lu rd_ok1=%lu rd_fail1=%lu rd_ok2=%lu rd_fail2=%lu max_us=%lu\n",
                  (unsigned long)tx_ok, (unsigned long)tx_fail,
                  (unsigned long)W[0].rd_ok, (unsigned long)W[0].rd_fail,
                  (unsigned long)W[1].rd_ok, (unsigned long)W[1].rd_fail,
                  (unsigned long)max_us);
    max_us = 0;
    lastBusPrintMs = now_ms;
  }
}
