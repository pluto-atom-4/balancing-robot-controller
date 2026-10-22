// ttl_servo: closed-loop Segway physics simulation for Wokwi, driving the
// simulated Dynamixel XL330 chip.
// SIM-ONLY, UNVERIFIED on hardware. The firmware simulates the Segway frame
// (lib/segway_sim) and balances it with a PD loop. The real MPU6050 gyro Z is
// a disturbance rate injected into the simulated tilt dynamics (move the
// Wokwi gyro Z slider to push the frame). The IMU I2C pins GPIO8/9 are a sim-only
// Wokwi wiring choice (as in stepper_motor), not the hardware pins (tilt_servo/balance use GPIO6/7).
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
// A failed bring-up also stops then latches, and always forces torque off.
// -D TTL_SERVO_HORN_SELFTEST: after gyro calibration, 2 s forward + 2 s reverse at 90 deg/s, then 0.
// -D TTL_SERVO_FALL_TEST: low gains (kp=2 kd=1) so the frame falls within seconds.
// -D TTL_SERVO_KILL_POC: SIM-ONLY disposable kill-switch POC for #148 (GPIO3 button, active low).
// Delete with #151; limits: see kill_poc_poll.
// -D TTL_SERVO_DOF_FRAMES: SIM-ONLY synthetic IMU frame stream (#187, epic #184). Gravity-only
// accel and pitch-rate gyro from SegwaySim theta (lib/segway_imu, signs UNVERIFIED per #185),
// not hardware-validated. Frames are the sim state, never the chip's accel or gyro. The body
// rests upright for kDofRestTicks ticks, then the PD sim runs. The stream stops on a fall.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <segway_sim.h>
#if defined(TTL_SERVO_DOF_FRAMES) && defined(TTL_SERVO_IMU_FRAMES)
#error "TTL_SERVO_DOF_FRAMES and TTL_SERVO_IMU_FRAMES are mutually exclusive: define only one"
#endif
#if defined(TTL_SERVO_IMU_FRAMES) || defined(TTL_SERVO_DOF_FRAMES)
// Shared frame block: POC raw-gyro frames (TTL_SERVO_IMU_FRAMES) and the DOF sim stream
// (TTL_SERVO_DOF_FRAMES). The POC-only text below is the TTL_SERVO_IMU_FRAMES note.
// -D TTL_SERVO_IMU_FRAMES: SIM-ONLY disposable POC (#160), raw IMU frames on USB CDC.
//   Delete with the removal issue (#166).
//   One IMU,<seq>,<t_us>,... frame per loop tick with a good IMU read (format: lib/imu_frame).
//   RAW accel (g) and gyro (deg/s): no bias removal, no filter, never from segway theta.
//   ttl_servo sets the gyro range to 250 deg/s; the accel range is the library default (2 g).
//   Frame size is an ESTIMATE, not measured: about 65-95 bytes depending on digit counts.
//   -D TTL_SERVO_HORN_SELFTEST blocks boot, so no frames are sent during the self-test.
//   The combination with -D TTL_SERVO_KILL_POC is untested.
//   Non-finite values are NOT checked here: a NaN/inf gyro or accel goes out as 0.000000
//   with a valid checksum (the formatter maps non-finite to 0; Python dof_frame.py raises).
//   The stream keeps the CDC TX buffer closer to full, so the existing unguarded Serial.printf
//   lines (10 Hz sim line, 1 Hz [dxl] line) are more likely to block if the host stalls or is
//   absent. Documented, not fixed: no setTxTimeoutMs is set.
#include <imu_frame.h>
#endif
#ifdef TTL_SERVO_DOF_FRAMES
#include <segway_imu.h>  // SIM-ONLY DOF stream synthesis (#186), signs UNVERIFIED (#185)
#endif

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
#ifdef TTL_SERVO_KILL_POC
static constexpr int      KILL_PIN = 3;                // SIM-ONLY POC kill button (active low)
static constexpr uint8_t  KILL_DEBOUNCE_TICKS = 2;     // consecutive LOW loop ticks
#endif

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
#ifdef TTL_SERVO_KILL_POC
static uint8_t kill_low_ticks = 0;   // consecutive LOW ticks on KILL_PIN
static bool kill_latched = false;    // POC kill pressed: no further polling
#endif

#if defined(TTL_SERVO_IMU_FRAMES) || defined(TTL_SERVO_DOF_FRAMES)
#ifdef TTL_SERVO_IMU_FRAMES
static constexpr float    kGravityMs2 = 9.80665f;    // m/s^2 per g (frame accel scale)
#endif
static constexpr uint32_t FRAMES_PRINT_PERIOD_MS = 1000;  // 1 Hz [frames] counters line
// Free TX bytes needed before the [frames] line is printed (worst case line is 67 bytes).
static constexpr int      kFramesLineRoom = 80;
static uint16_t frame_seq = 0;          // advances on every attempted frame, sent or dropped
static uint32_t frames_sent = 0;
static uint32_t frames_dropped = 0;
static uint32_t frames_max_emit_us = 0; // worst frame format+write time since the last print
static uint32_t lastFramesPrintMs = 0;
static char frame_buf[128];             // one frame, NUL included (about 65-95 bytes used)
#endif
#ifdef TTL_SERVO_DOF_FRAMES
static constexpr int      kDofRestTicks = 150;  // 3 s at 50 Hz, counted in loop ticks (not ms)
static int dof_rest_ticks = 0;          // physics frozen while below kDofRestTicks
static bool dof_stream_stopped = false; // latched on a fall: no more DOF frames until reset
#endif

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

#if defined(TTL_SERVO_IMU_FRAMES) || defined(TTL_SERVO_DOF_FRAMES)
// Formats one frame from a_g/g_dps and writes it only if the whole frame fits in the free
// CDC TX space (one write, no blocking). seq advances on every attempt, sent or dropped.
static void frames_emit(const float a_g[3], const float g_dps[3]) {
  const uint32_t t0 = micros();
  const size_t n = imu_frame::format(frame_buf, sizeof frame_buf, frame_seq, t0, a_g, g_dps);
  // avail is an int: a negative value would convert to a huge size_t and pass the
  // >= n test, so the avail > 0 guard stays.
  const int avail = Serial.availableForWrite();
  if (n > 0 && avail > 0 && static_cast<size_t>(avail) >= n) {
    Serial.write(reinterpret_cast<const uint8_t*>(frame_buf), n);
    ++frames_sent;
  } else {
    ++frames_dropped;
  }
  ++frame_seq;  // every attempted frame advances seq, so a host sees gaps on drops
  const uint32_t el = micros() - t0;
  if (el > frames_max_emit_us) frames_max_emit_us = el;
}
#endif

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
  // Torque-off transactions are not counted in tx_ok/tx_fail/max_us.
  // Bring-up ping and config calls are not counted either.
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
// chip's present position every 250 ms. Blocks boot (~4 s) with the wheel spinning and
// no fall check active; needs a live servo. A failed write stops the chip and latches
// bus_down at once (no 3-in-a-row rule here). Keep the flag out of non-sim builds.
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
      if (selftest_read_pos(&pos)) {
        if (have_prev) {
          Serial.printf("[ttl_servo] selftest cmd=%ld pos=%ld dpos=%ld\n", (long)cmds[ph],
                        (long)pos, (long)(((pos - prev + 6144) % 4096) - 2048));
        } else {
          Serial.printf("[ttl_servo] selftest cmd=%ld pos=%ld dpos=n/a\n", (long)cmds[ph], (long)pos);
        }
        prev = pos;
        have_prev = true;
      } else {
        Serial.printf("[ttl_servo] selftest cmd=%ld pos=ERR\n", (long)cmds[ph]);
        have_prev = false;
      }
    }
  }
  have_last_pos = false;  // telemetry dpos restarts from its first read
}
#endif

#ifdef TTL_SERVO_KILL_POC
// SIM-ONLY POC: debounced kill button, polled from loop() only (not setup()).
// Latches bus_down through the existing stop + latch helpers; no new transaction types
// (the kill path adds call sites that reuse stop_chip_best_effort and latch_bus_down).
// Limits:
// - Polled from loop() only. pinMode(INPUT_PULLUP) runs at the end of setup(), after
//   gyro calibration and HORN_SELFTEST. A press released before loop() starts polling
//   is missed (boot, gyro cal, HORN_SELFTEST). A press still held at loop start is
//   seen after KILL_DEBOUNCE_TICKS ticks (about 40 ms).
// - A press shorter than the debounce window (KILL_DEBOUNCE_TICKS x LOOP_PERIOD_MS,
//   about 40 ms) is not guaranteed to be seen, at any time.
// - If bus_down is already latched (for example after 3 failed writes), the stop before
//   that latch sends velocity 0 and tries torque off only if the zero write fails, so
//   torque may still be on. A press then only prints; the kill path makes NO
//   Dynamixel calls at all.
static void kill_poc_poll() {
  if (kill_latched) return;
  if (digitalRead(KILL_PIN) == LOW) {
    if (kill_low_ticks < KILL_DEBOUNCE_TICKS) ++kill_low_ticks;
  } else {
    kill_low_ticks = 0;
  }
  if (kill_low_ticks < KILL_DEBOUNCE_TICKS) return;
  kill_latched = true;
  if (!bus_down) {
    Serial.println("[safe] KILL switch pressed (sim POC)");
    (void)stop_chip_best_effort("kill switch", true);
    latch_bus_down();
  } else {
    Serial.println("[safe] KILL switch pressed (bus already down)");
  }
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
    if (!bus_down) (void)stop_chip_best_effort("gyro cal failed", true);
    while (1) { delay(10); }
  }
  gyroZ_bias_rad_s = sum / (float)good;
  Serial.printf("[ttl_servo] gyro Z bias=%.4f rad/s (keep the slider at 0 during boot)\n",
                gyroZ_bias_rad_s);
#ifdef TTL_SERVO_IMU_FRAMES
  Serial.println("[frames] on raw-gyro");
#endif
#ifdef TTL_SERVO_DOF_FRAMES
  Serial.println("[frames] on dof-sim");
#endif

#ifdef TTL_SERVO_HORN_SELFTEST
  horn_selftest();
#endif
#ifdef TTL_SERVO_DOF_FRAMES
  segway_sim.reset(0.0f);  // DOF: start upright for the rest phase
#else
  segway_sim.reset(SIM_INIT_THETA_RAD);
#endif
#ifdef TTL_SERVO_FALL_TEST
  segway_sim.params.kp = 2.0f;
  segway_sim.params.kd = 1.0f;
  Serial.println("[ttl_servo] FALL TEST: kp=2 kd=1 (expect FELL within ~6 s)");
#endif
#ifdef TTL_SERVO_KILL_POC
  pinMode(KILL_PIN, INPUT_PULLUP);
#endif
#ifdef TTL_SERVO_DOF_FRAMES
  Serial.println("[ttl_servo] Sim starts upright at 0 deg for the DOF rest phase (~3 s), then PD. Move gyro Z to push.\n");
#else
  Serial.println("[ttl_servo] Sim starts at 15 deg; PD should settle to 0. Move gyro Z to push.\n");
#endif

  lastLoopMs = millis();
  lastPrintMs = lastLoopMs;
  lastBusPrintMs = lastLoopMs;
#if defined(TTL_SERVO_IMU_FRAMES) || defined(TTL_SERVO_DOF_FRAMES)
  lastFramesPrintMs = lastLoopMs;
#endif
}

void loop() {
  uint32_t now_ms = millis();
  uint32_t dt_ms = now_ms - lastLoopMs;
  if (dt_ms < LOOP_PERIOD_MS) {
    delay(1);
    return;
  }
  lastLoopMs = now_ms;

#ifdef TTL_SERVO_KILL_POC
  kill_poc_poll();
#endif

  // 1. Real gyro Z (bias removed, filtered) is the disturbance rate.
  sensors_event_t accel, gyro, temp;
#ifdef TTL_SERVO_IMU_FRAMES
  bool imu_ok = false;  // this tick's IMU read succeeded (frame block after step 4)
#endif
  // On a failed read keep the previous filtered value rather than stale data.
  if (mpu.getEvent(&accel, &gyro, &temp)) {
#ifdef TTL_SERVO_IMU_FRAMES
    imu_ok = true;
#endif
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
#ifdef TTL_SERVO_DOF_FRAMES
    // DOF rest phase: physics frozen at theta 0 with the wheel at 0 (no servo write).
    const bool dof_rest = dof_rest_ticks < kDofRestTicks;
    if (dof_rest) ++dof_rest_ticks;
#else
    const bool dof_rest = false;
#endif
    if (!dof_rest) (void)segway_sim.tick_cmd(gyroZ_filtered_rad_s, dt_s);

    if (segway_sim.has_fallen) {
      // 4. Fall first, so a fall tick never writes the lagged velocity.
      Serial.printf("[ttl_servo] FELL theta=%.3f\n", segway_sim.theta_rad);
#ifdef TTL_SERVO_DOF_FRAMES
      if (!dof_stream_stopped) {
        dof_stream_stopped = true;
        Serial.println("[frames] stopped (fall)");
      }
#endif
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

#ifdef TTL_SERVO_IMU_FRAMES
  // 4b. SIM-ONLY raw IMU frame (#160, #163). Emitted only when this tick's IMU read succeeded.
  // RAW values: no bias subtraction, no EMA, never from segway theta. Scales: accel is divided
  // by kGravityMs2 (g); gyro is multiplied by segway::kRadToDeg (deg/s). The gyro range is set to
  // 250 deg/s in setup(); the accel range is never set here (Adafruit library default, 2 g).
  // tilt_servo uses 500 deg/s and 4 g, so raw scales differ between the two POCs.
  if (imu_ok) {
    const float a_g[3] = {accel.acceleration.x / kGravityMs2,
                          accel.acceleration.y / kGravityMs2,
                          accel.acceleration.z / kGravityMs2};
    const float g_dps[3] = {gyro.gyro.x * segway::kRadToDeg, gyro.gyro.y * segway::kRadToDeg,
                            gyro.gyro.z * segway::kRadToDeg};
    frames_emit(a_g, g_dps);  // t_us is the emit time, not the IMU sample time
  }
#endif
#ifdef TTL_SERVO_DOF_FRAMES
  // 4c. SIM-ONLY synthetic DOF stream (#187, #186). Built from SegwaySim state after the step,
  // never from the chip's or the MPU's accel/gyro (gz comes out 0). Not on a fall tick
  // (holding is set there), not while holding, not after the stream is stopped by a fall.
  // Signs UNVERIFIED (#185); not hardware-validated.
  if (!holding && !dof_stream_stopped) {
    float a_g[3], g_dps[3];
    segway_imu::from_state(segway_sim.theta_rad, segway_sim.theta_dot_rad_s, a_g, g_dps);
    frames_emit(a_g, g_dps);
  }
#endif

#if defined(TTL_SERVO_IMU_FRAMES) || defined(TTL_SERVO_DOF_FRAMES)
  // 1 Hz counters line. Outside the servo_ok [dxl] block, so it prints with no servo.
  // Gated on free TX space (kFramesLineRoom): with less room the print is skipped and the
  // counters and the timer are NOT reset, so it prints at the next tick that has room.
  // Not a no-blocking guarantee: unguarded prints can still block (see the imu_frame.h note).
  if (now_ms - lastFramesPrintMs >= FRAMES_PRINT_PERIOD_MS &&
      Serial.availableForWrite() >= kFramesLineRoom) {
    Serial.printf("[frames] sent=%lu dropped=%lu max_emit_us=%lu\n",
                  (unsigned long)frames_sent, (unsigned long)frames_dropped,
                  (unsigned long)frames_max_emit_us);
    frames_max_emit_us = 0;
    lastFramesPrintMs = now_ms;
  }
#endif

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
    // During the post-fall hold the plant is frozen. The wheel is shown as 0 whether or not
    // the stop was confirmed; cmd shows the last confirmed goal and bus shows liveness
    // (a failed stop leaves the chip on its last goal).
    float wheel_shown = holding ? 0.0f : segway_sim.wheel_vel_rad_s;
    Serial.printf("[ttl_servo] sim theta=%.3f rate=%.3f wheel=%.2f cmd=%ld pos=%s dpos=%s gyroZ=%.3f bus=%d\n",
                  segway_sim.theta_rad, segway_sim.theta_dot_rad_s,
                  wheel_shown, (long)last_written_raw,
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
