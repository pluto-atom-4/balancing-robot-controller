// c3_ctrl_wheels: twin-wheel Wokwi program, derived from programs/ttl_servo. SIM-ONLY, UNVERIFIED,
// not hardware-validated. No balance claim.
// Both wheels: two Dynamixel XL330 chips, ID 1 (left, W[0]) and ID 2 (right, W[1], unconfirmed), on one servo bus,
// Serial1 GPIO20 (RX) / GPIO21 (TX), 57600 baud, protocol 2.0, through the FE-URT-1 adapter (direction
// handled in hardware), velocity mode. Each wheel has its own lib/segway_sim plant, ticked with the same
// MPU6050 gyro Z disturbance and dt. Each goal is that plant's lagged wheel velocity in raw units, capped
// on the host; the wheel sign is applied in write_goal_raw only.
// A tick is failed if either write fails. 3 failed ticks in a row: forced torque-off on both ids (zero
// velocity first), then latch bus_down (no Dynamixel calls; reset the board to retry).
// Fall (either plant): zero-velocity stop on both ids, torque off on both if a zero write failed; then
// hold 1 s and restart both. A failed stop latches bus_down.
// MPU6050 gyro Z is a disturbance rate injected into the sim tilt dynamics. Its IMU I2C is
// on GPIO8/9, a sim-only Wokwi wiring choice (hardware tilt_servo and balance use GPIO6/7).
// It also prints one WHL2 line per 20 ms tick, WHL2,<seq>,<t_us>,<model_rad_s>,<rawL>,<rawR>,<state>*HH
// (format in lib/wheels_frame/wheels_frame.h; state 0 run, 1 hold after a fall, 2 bus down), for the
// freecad-workspace twin contract poc/ctrl-wheels (CONTRACT.md S1; the host parser is ctrl_wheels_msg.py
// there). Evidence so far is the owner's simulator runs only (headless wokwi-cli, VS Code Wokwi over
// RFC2217 port 4002): 773 WHL2 lines parsed without errors or seq gaps; nothing on hardware. Plus a
// 1 Hz [ctrl_wheel] sent= dropped= max_emit_us= line. model is the LEFT wheel's (W[0]) lagged MODEL
// velocity, not a measurement. rawL and rawR are the raw values last CONFIRMED on each chip, as written
// (signed; the sign is applied once, in write_goal_raw). An unknown chip state and a confirmed 0 both
// report 0, so a consumer cannot tell them apart: a 0 raw can mean unknown after a failed write. State 0
// does not mean the writes succeeded. State 2 (after 3 failed ticks latch) is the only fault signal in
// the line. The [dxl] write failed log is rate-limited (first failure and every 50th). State 2 reports
// 0 for both raws.
// This is a RELAY to Webots, NOT a closed loop through Webots: nothing is received from Webots.
// Log tags are [ttl_servo] and [dxl].
// -D TTL_SERVO_FALL_TEST: low gains (kp=2 kd=1) so the frame falls.
// RFC2217 port is set in wokwi.toml.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Dynamixel2Arduino.h>
#include <segway_sim.h>
#include <wheels_frame.h>

// ===== Configuration =====
static constexpr uint8_t  I2C_SDA = 8;
static constexpr uint8_t  I2C_SCL = 9;
static constexpr uint8_t  MPU_ADDR = 0x68;
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
// Twin wheels (SIM-ONLY, UNVERIFIED). The ONLY place the wheel signs are defined.
// The sign is applied in write_goal_raw() only. Goal writes go through write_goal_raw() (loop) or
// write_vel_zero_ctx() (stops and bring-up). IDs must match the servoId
// attribute of the servo chips in diagram.json.
// A servo chip with no servoId attribute answers as ID 1, so servo2 must carry servoId "2"
// or it collides with servo1.
static constexpr uint8_t SERVO_ID_LEFT    = 1;
static constexpr uint8_t SERVO_ID_RIGHT   = 2;   // unconfirmed
static constexpr int8_t  WHEEL_SIGN_LEFT  = 1;   // unconfirmed
static constexpr int8_t  WHEEL_SIGN_RIGHT = -1;  // unconfirmed mirror

// ===== Globals =====
Adafruit_MPU6050 mpu;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
uint32_t lastLoopMs = 0;
uint32_t lastPrintMs = 0;
float gyroZ_bias_rad_s = 0.0f;
float gyroZ_filtered_rad_s = 0.0f;
bool servo_ok = false;          // false: chip absent or setup failed, read-only sim
uint32_t write_fail = 0;
static uint32_t tx_ok, tx_fail, rd_ok, rd_fail, max_us;
static uint32_t lastBusPrintMs;
static int32_t last_pos = 0;
static bool have_last_pos = false;
static bool bus_down = false;        // latched: no Dynamixel calls until reset
static uint8_t tick_fail_streak = 0; // consecutive failed ticks (a tick fails if either wheel's write fails)
static bool holding = false;         // true for 1 s after a fall
static uint32_t hold_until_ms = 0;
// WHL2 stream (sim-only, UNVERIFIED). 96 is a sizing choice (the buffer includes the NUL). Worst case
// is about 61 bytes with int32 raws (11 chars each) and a 9-character model; real raws stay within
// +-125, so a typical line is about 47-48 bytes. wheels_frame::format returns 0 if it does not fit,
// and the line is then dropped. Not measured.
// whl_seq is uint16_t: it wraps to 0 after 65536 ticks (about 22 min at 50 Hz). t_us is uint32 micros(),
// which wraps after about 71.6 min. The host side must handle both wraps; this repo's tests do not show that.
static uint16_t whl_seq = 0;
static uint32_t whl_sent = 0, whl_dropped = 0, whl_max_emit_us = 0, lastWhlPrintMs = 0;
static char whl_buf[96];
// Per-wheel state for the twin: W[0] is the left wheel (ID 1), W[1] the right (ID 2).
// In use: id, sign, sim (each wheel's own plant), last_raw.
// Not used: last_pos, have_last_pos, rd_ok, rd_fail. The 10 Hz read telemetry still uses the
// globals of the same names.
struct WheelCtx {
  uint8_t  id;
  int8_t   sign;
  segway::SegwaySim sim;
  uint8_t  consec_fail;     // per-wheel consecutive failure count, saturates at 255; not used for the latch (tick_fail_streak is)
  int32_t  last_raw;       // last raw CONFIRMED on the chip (already signed); INT32_MIN = chip state unknown (initial value, and after any failed or unconfirmed step), so the next write_goal_raw always goes out
  int32_t  last_pos;        // unused, see above
  bool     have_last_pos;   // unused, see above
  uint32_t rd_ok, rd_fail;  // unused, see above
  WheelCtx(uint8_t id_, int8_t sign_)
    : id(id_), sign(sign_), sim(SIM_INIT_THETA_RAD), consec_fail(0),
      last_raw(INT32_MIN), last_pos(0), have_last_pos(false), rd_ok(0), rd_fail(0) {}
};
static WheelCtx W[2] = { WheelCtx(SERVO_ID_LEFT, WHEEL_SIGN_LEFT),
                         WheelCtx(SERVO_ID_RIGHT, WHEEL_SIGN_RIGHT) };

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


// Raw reported in WHL2 for one wheel: the value last CONFIRMED on its chip (already signed, see
// write_goal_raw). Unknown (INT32_MIN: before the first write, or after a failed write or any
// torque-off attempt) is reported as 0.
static int32_t whl2_raw(const WheelCtx& w) {
  return (w.last_raw == INT32_MIN) ? 0 : w.last_raw;
}

// One WHL2 line per call. Written only if availableForWrite() reports room for the whole line.
// Whether Serial.write can still block on the C3 USB CDC is NOT verified, and the loop's other
// Serial prints (for example the 10 Hz line, the [dxl] lines and the FELL/restart prints) are
// unguarded, so they can block when the host stalls.
// wheels_frame::format also returns 0 for a non-finite model; that counts as dropped and whl_seq still advances.
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

// Twin helpers (SIM-ONLY, UNVERIFIED, not hardware-validated). tx_ok, tx_fail and max_us are shared
// by all wheels. consec_fail is per-wheel (not used for the latch). The write-failure log line (`[dxl] write failed`,
// write_fail) and the failed-tick latch live in loop(), not here.
// Sign: see the note at the top (config block), applied only in write_goal_raw(). Do not multiply by .sign anywhere else.
// Rule (see here, referred to in other comments): last_raw is the last value CONFIRMED on the chip;
// INT32_MIN means the chip state is unknown, so the next write for that wheel always goes out.

// Zero goal velocity to w.id, up to 3 attempts. On success w.last_raw = 0 (confirmed);
// if all 3 tries fail, w.last_raw = INT32_MIN (unknown, see the rule in the block header).
// consec_fail is a per-wheel consecutive failure count (not used for the latch) and saturates at 255.
// Not gated by bus_down: this is part of the kill/torque-off path, which must still try after a latch.
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

// Goal write for one wheel, called from loop(). raw is the UNSIGNED forward raw from
// segway::wheel_cmd_to_raw, already capped at +-segway::kWheelCapRaw; do not re-cap here.
// The wire value is w.sign * raw, applied here and nowhere else.
// Write on change: no bus call if the signed value equals w.last_raw (the last value confirmed on the chip).
// A failed write sets w.last_raw = INT32_MIN, so the next call for that wheel always goes out
// (a timed-out write may have landed; see the rule in the block header).
// Refused after the bus latch: a normal command write must not go out once bus_down is set.
// Does NOT latch. The tick rule (either write fails = failed tick; 3 in a row latch both) is in loop().
static bool write_goal_raw(WheelCtx& w, int32_t raw) {
  if (bus_down) return false;
  int32_t wire = (int32_t)w.sign * raw;
  if (wire == w.last_raw) return true;
  uint32_t t0 = micros();
  bool ok = dxl.setGoalVelocity(w.id, (float)wire, UNIT_RAW);
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

// Stops BOTH wheels, W[0] then W[1]; both are always attempted. Zero velocity with up to 3 tries per
// wheel (write_vel_zero_ctx). Then, if any zero write failed or force_torque_off, torque off on EACH id
// (up to 3 tries each, both ids always attempted). Not gated by bus_down: this is the kill/torque-off
// path, which must still try after a latch. Does not latch. Prints one line per wheel (stop) and one
// line per id (torque off).
// Return: non-force returns true only if both zero writes were ok (torque off is then not sent).
// Force returns the torque-off result only (off_all), ignoring the zero-write result, because
// torque-off is the stronger stop.
// Cost on a dead bus (unmeasured, sim): up to 12 transactions (6 zero writes, 6 torque-offs) before
// the caller latches.
// Every torque-off attempt (confirmed or not) sets last_raw = INT32_MIN for that wheel: the chip
// state is unknown, and no real signed raw equals INT32_MIN (|raw| <= 125), so the next write_goal_raw
// goes out, including after torque is re-enabled (see the rule in the block header).
static bool stop_wheels_best_effort(const char* why, bool force_torque_off = false) {
  bool zok_all = true;
  for (uint8_t k = 0; k < 2; ++k) {
    bool zok = write_vel_zero_ctx(W[k]);
    Serial.printf("[dxl] stop id=%lu vel=0 ok=%d\n", (unsigned long)W[k].id, zok ? 1 : 0);
    if (!zok) zok_all = false;
  }
  if (zok_all && !force_torque_off) return true;
  // Torque-off transactions are not counted in tx_ok/tx_fail/max_us.
  bool off_all = true;
  for (uint8_t k = 0; k < 2; ++k) {
    bool off = false;
    for (uint8_t i = 0; i < 3 && !off; ++i) off = dxl.torqueOff(W[k].id);
    Serial.printf("[safe] torque off %s id=%lu%s\n", why, (unsigned long)W[k].id,
                  off ? "" : " (TORQUE OFF NOT CONFIRMED)");
    if (!off) off_all = false;
    W[k].last_raw = INT32_MIN;  // torque-off attempt: chip state unknown, next write goes out
  }
  return force_torque_off ? off_all : false;
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

// Bring-up for one wheel, in the order the chip needs: ping (fills the model that
// Dynamixel2Arduino uses to resolve registers), torque off, velocity mode, goal 0, torque on.
// Each step runs only if the previous one passed. On any failure: best-effort torque off on
// this id, and w.last_raw = INT32_MIN (chip state unknown). Runs at boot, before any latch.
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

// Both plants back to the initial lean. The -D TTL_SERVO_FALL_TEST gains are applied at boot and on
// every restart (this helper runs in both places).
static void reset_sims() {
  for (uint8_t k = 0; k < 2; ++k) {
    W[k].sim.reset(SIM_INIT_THETA_RAD);
#ifdef TTL_SERVO_FALL_TEST
    W[k].sim.params.kp = 2.0f;
    W[k].sim.params.kd = 1.0f;
#endif
  }
}

// ===== Arduino entry points =====
void setup() {
  Serial.begin(115200);
  delay(1000); // Boot without blocking on USB host

  dxl_bus_begin();
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("\n[ttl_servo] Segway physics sim (Wokwi, SIM-ONLY, UNVERIFIED)");

  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("[ttl_servo] Failed to find MPU6050 chip");
    // Ping both ids first: Dynamixel2Arduino resolves registers through the model number a ping fills in,
    // so a write without a prior ping fails even when the servo is present.
    for (uint8_t k = 0; k < 2; ++k) (void)dxl.ping(W[k].id);
    (void)stop_wheels_best_effort("no IMU", true);
    while (1) { delay(10); }
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Servo bring-up before the (blocking) calibration; the bus was begun above.
  // Both wheels in order, stopping at the first failure: the stop below forces torque off on
  // both ids and the bus latches. servo_ok only when both wheels came up.
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
    // Boot-block timing, unmeasured estimate (sim). Worst case: ID 1 comes up (5 transactions), then
    // ID 2 fails at goal 0 with every retry timing out (7: ping, torque off, mode, goal 0 with 3 tries,
    // best-effort torque off; torque on is unreachable after a failed goal 0). The forced stop of both
    // ids is up to 3 zero writes per wheel (6) and up to 3 torque-offs per id (6), 12. About 24
    // transactions (5 + 7 + 12), roughly 480 ms at an assumed 20 ms per timeout.
    (void)stop_wheels_best_effort("bring-up failed", true);
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

// loop() drives both wheels (ID 1 left, ID 2 right, unconfirmed) and stops both on a fall or a latch.
// A tick fails if either goal write fails; 3 failed ticks in a row force a stop of both ids, then
// latch. Not exercised in the sim: the 3-failed-tick latch and bus-down state 2.
// Ticking two plants per loop: the soft-float cost is unmeasured (sim).
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
  // On a failed read keep the previous filtered value rather than a bogus sample.
  if (mpu.getEvent(&accel, &gyro, &temp)) {
    float gz = gyro.gyro.z - gyroZ_bias_rad_s;
    gyroZ_filtered_rad_s = ema_update(gyroZ_filtered_rad_s, gz, dt_ms);
  } else {
    Serial.println("[ttl_servo] IMU read failed, holding last gyro Z");
  }

  // 2. Hold after a fall: physics frozen, wheel stays stopped, then restart.
  if (holding) {
    if ((int32_t)(now_ms - hold_until_ms) >= 0) {
      reset_sims();
      holding = false;
      tick_fail_streak = 0;
      Serial.println("[ttl_servo] restart");
    }
  } else {
    // 3. PD -> physics, same code the native tests exercise, once per plant with the same gyro Z
    // and dt. The returned PD commands are ignored: each chip gets its plant's lagged wheel velocity.
    float dt_s = clamp_dt_s(dt_ms);
    for (uint8_t k = 0; k < 2; ++k) (void)W[k].sim.tick_cmd(gyroZ_filtered_rad_s, dt_s);

    if (W[0].sim.has_fallen || W[1].sim.has_fallen) {
      // 4. Fall first, so a fall tick never writes the lagged velocity. Either plant falling stops both.
      uint8_t f = W[0].sim.has_fallen ? 0 : 1;
      Serial.printf("[ttl_servo] FELL id=%lu theta=%.3f\n",
                    (unsigned long)W[f].id, W[f].sim.theta_rad);
      // Non-force stop: torque off on both ids only if a zero write failed; a stop where both zero
      // writes succeed leaves torque on at zero velocity.
      // After a latch (servo_ok false) no stop is sent; the latch already forced torque off on both ids.
      // The 1 s hold and restart still run, and state stays 2.
      if (servo_ok) {
        if (!stop_wheels_best_effort("after fall")) latch_bus_down();
      }
      hold_until_ms = millis() + 1000;
      holding = true;
    } else if (servo_ok) {
      // Unsigned raw per wheel; the sign is applied in write_goal_raw. Both writes are always
      // attempted. A tick fails if either write fails. A failed wheel keeps last_raw = INT32_MIN,
      // so it retries on every tick; a tick with no bus call (write on change) is not a failure,
      // so it resets the streak.
      int32_t raw0 = segway::wheel_cmd_to_raw(W[0].sim.wheel_vel_rad_s);
      int32_t raw1 = segway::wheel_cmd_to_raw(W[1].sim.wheel_vel_rad_s);
      bool ok0 = write_goal_raw(W[0], raw0);
      bool ok1 = write_goal_raw(W[1], raw1);
      if (ok0 && ok1) {
        tick_fail_streak = 0;
      } else {
        ++write_fail;
        // raw in this log is the UNSIGNED value passed to write_goal_raw; the wire value is sign * raw.
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
          // Forced torque-off on both ids, then the latch.
          (void)stop_wheels_best_effort("before bus latch", true);
          latch_bus_down();
        }
      }
    }
  }

  // 4b. WHL2 line, once per tick (after the 20 ms gate, outside the servo_ok chip branch).
  // model is the left wheel's (W[0]) lagged model, 0 while holding. The raws are the values last
  // confirmed on each chip (signed as written); unknown is 0, and state 2 reports 0 for both.
  const float whl_model = holding ? 0.0f : W[0].sim.wheel_vel_rad_s;
  const uint8_t whl_state = bus_down ? 2 : (holding ? 1 : 0);
  const int32_t whl_rawL = (whl_state == 2) ? 0 : whl2_raw(W[0]);
  const int32_t whl_rawR = (whl_state == 2) ? 0 : whl2_raw(W[1]);
  whl_emit(whl_model, whl_rawL, whl_rawR, whl_state);

  // 5. Telemetry (10 Hz)
  if (now_ms - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now_ms;
    char pos_s[12] = "NA";
    char dpos_s[12] = "n/a";
    char cmd_s[12] = "NA";
    if (servo_ok) {
      uint32_t t0 = micros();
      int32_t pos = dxl.getPresentPosition(W[0].id);
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
    // pos and dpos are read from W[0] (left, SERVO_ID_LEFT) only; the read telemetry keeps the globals
    // last_pos, have_last_pos, rd_ok and rd_fail. cmd = W[0].last_raw, the last value CONFIRMED on
    // the left wheel; NA when that state is unknown (after a failed write or stop). bus=1 means both wheels
    // came up at bring-up (servo_ok); bus=0 means bring-up failed or the bus is latched.
    // During the post-fall hold the plants are frozen. The wheel is shown as 0 whether or not
    // the stop was confirmed.
    if (W[0].last_raw != INT32_MIN)
      snprintf(cmd_s, sizeof(cmd_s), "%ld", (long)W[0].last_raw);
    float wheel_shown = holding ? 0.0f : W[0].sim.wheel_vel_rad_s;
    Serial.printf("[ttl_servo] sim theta=%.3f rate=%.3f wheel=%.2f cmd=%s pos=%s dpos=%s gyroZ=%.3f bus=%d\n",
                  W[0].sim.theta_rad, W[0].sim.theta_dot_rad_s,
                  wheel_shown, cmd_s,
                  pos_s, dpos_s, gyroZ_filtered_rad_s, servo_ok ? 1 : 0);
  }

  // Gate: 80 bytes of USB TX room, a sizing choice for this 1 Hz [ctrl_wheel] line (up to about 71
  // bytes with 10-digit counters; not measured). The WHL2 line is gated separately in whl_emit.
  if (now_ms - lastWhlPrintMs >= 1000 && Serial.availableForWrite() >= 80) {
    Serial.printf("[ctrl_wheel] sent=%lu dropped=%lu max_emit_us=%lu\n",
                  (unsigned long)whl_sent, (unsigned long)whl_dropped,
                  (unsigned long)whl_max_emit_us);
    whl_max_emit_us = 0;
    lastWhlPrintMs = now_ms;
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
