// stepper_motor: closed-loop Segway physics simulation driving a stepper (Wokwi).
// SIM-ONLY, UNVERIFIED on hardware. lib/segway_sim simulates the frame and a PD
// loop balances it; the real MPU6050 gyro Z (Wokwi slider) is a disturbance
// rate. The simulated wheel velocity sets the A4988 step rate and direction,
// so the motor spins continuously at a speed that follows the PD loop.
//
// Timing: the control tick (I2C read + sim) runs at 50 Hz; the step service
// runs on every loop pass and emits whole steps from a polled accumulator.
// Steps demanded beyond kMaxStepsPerPass in one pass are dropped and counted
// (telemetry "dropped"), never hidden. Scale is physical 1:1 at 1/16
// microstepping (MS1-3 tied high in diagram.json): 3 rad/s ~ 1528 steps/s.
// Whether Wokwi sustains that pulse rate is unverified.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <segway_sim.h>
#include <stepper_speed.h>

// ===== Configuration =====
static constexpr uint8_t  STEP_PIN = 3;
static constexpr uint8_t  DIR_PIN = 2;
static constexpr uint8_t  I2C_SDA = 8;
static constexpr uint8_t  I2C_SCL = 9;
static constexpr uint8_t  MPU_ADDR = 0x68;
static constexpr uint32_t I2C_HZ = 400000;
static constexpr float    STEPS_PER_REV = 3200.0f;      // 200 x 1/16 microstep
static constexpr unsigned kMaxStepsPerPass = 8;
static constexpr bool     DIR_FORWARD_LEVEL = HIGH;     // DIR level for forward wheel
static constexpr uint32_t CONTROL_PERIOD_MS = 20;       // 50 Hz control tick
static constexpr uint32_t PRINT_PERIOD_MS = 100;        // 10 Hz serial output
static constexpr float    SIM_INIT_THETA_RAD = 0.262f;  // start leaning 15 deg
static constexpr float    GYRO_FILTER_TAU_MS = 50.0f;
static constexpr int      GYRO_CAL_SAMPLES = 100;       // ~1 s at boot, slider at rest

// ===== Globals =====
Adafruit_MPU6050 mpu;
segway::SegwaySim segway_sim(SIM_INIT_THETA_RAD);
stepper::StepAccumulator steps;
uint32_t lastControlMs = 0;
uint32_t lastPrintMs = 0;
uint32_t lastStepUs = 0;
float gyroZ_bias_rad_s = 0.0f;
float gyroZ_filtered_rad_s = 0.0f;
float stepRate = 0.0f;  // steps/s, signed; updated each control tick

// ===== Helpers =====
float ema_update(float prev, float raw, uint32_t dt_ms) {
  if (dt_ms == 0) return prev;
  float alpha = (float)dt_ms / (GYRO_FILTER_TAU_MS + (float)dt_ms);
  return prev * (1.0f - alpha) + raw * alpha;
}

// Measured dt in seconds, clamped so a stall cannot blow up the integrator.
float clamp_dt_s(uint32_t dt_ms) {
  float dt = (float)dt_ms / 1000.0f;
  if (dt < 0.005f) dt = 0.005f;
  if (dt > 0.1f) dt = 0.1f;
  return dt;
}

// Emit n STEP pulses (A4988 needs >= 1 us high and low).
void pulse_steps(unsigned n) {
  for (unsigned i = 0; i < n; ++i) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(2);
  }
}

// ===== Arduino entry points =====
void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, DIR_FORWARD_LEVEL);

  Serial.begin(115200);
  delay(1000);  // Boot without blocking on USB host
  Serial.println("\n[stepper] Segway physics sim -> stepper (Wokwi, SIM-ONLY, UNVERIFIED)");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("[stepper] Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

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
    Serial.println("[stepper] gyro calibration failed (IMU reads), halting");
    while (1) { delay(10); }
  }
  gyroZ_bias_rad_s = sum / (float)good;
  Serial.printf("[stepper] gyro Z bias=%.4f rad/s (keep the slider at 0 during boot)\n",
                gyroZ_bias_rad_s);

  segway_sim.reset(SIM_INIT_THETA_RAD);
  Serial.println("[stepper] Sim starts at 15 deg; PD settles to 0, motor speed follows the wheel.\n");

  lastControlMs = millis();
  lastPrintMs = lastControlMs;
  lastStepUs = micros();
}

void loop() {
  // 1. Step service on every pass: emit whole steps for the elapsed time.
  uint32_t now_us = micros();
  float step_dt = (float)(now_us - lastStepUs) / 1.0e6f;
  lastStepUs = now_us;
  unsigned n = steps.advance(stepRate, step_dt, kMaxStepsPerPass);
  if (n > 0) {
    digitalWrite(DIR_PIN, steps.forward() ? DIR_FORWARD_LEVEL : !DIR_FORWARD_LEVEL);
    pulse_steps(n);
  }

  // 2. Control tick at 50 Hz.
  uint32_t now_ms = millis();
  uint32_t dt_ms = now_ms - lastControlMs;
  if (dt_ms < CONTROL_PERIOD_MS) return;
  lastControlMs = now_ms;

  sensors_event_t accel, gyro, temp;
  // On a failed read keep the previous filtered value rather than stale data.
  if (mpu.getEvent(&accel, &gyro, &temp)) {
    float gz = gyro.gyro.z - gyroZ_bias_rad_s;
    gyroZ_filtered_rad_s = ema_update(gyroZ_filtered_rad_s, gz, dt_ms);
  } else {
    Serial.println("[stepper] IMU read failed, holding last gyro Z");
  }

  // PD wheel command (unquantised) -> physics -> step rate from the wheel's real velocity.
  float dt_s = clamp_dt_s(dt_ms);
  segway_sim.step(segway::pd_wheel_cmd(segway_sim.params, segway_sim.theta_rad,
                                       segway_sim.theta_dot_rad_s),
                  segway::disturbance_from_gyro(segway_sim.params, gyroZ_filtered_rad_s),
                  dt_s);
  stepRate = stepper::rad_s_to_steps_s(segway_sim.wheel_vel_rad_s, STEPS_PER_REV);

  // Fall: stop the motor, report, restart the sim.
  if (segway_sim.has_fallen) {
    Serial.printf("[stepper] FELL theta=%.3f, reset\n", segway_sim.theta_rad);
    segway_sim.reset(SIM_INIT_THETA_RAD);
    stepRate = 0.0f;
    steps.hold();
  }

  // 3. Telemetry (10 Hz)
  if (now_ms - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now_ms;
    Serial.printf("[stepper] theta=%.3f rate=%.3f wheel=%.2f sps=%.0f pos=%ld dropped=%lu gyroZ=%.3f\n",
                  segway_sim.theta_rad, segway_sim.theta_dot_rad_s,
                  segway_sim.wheel_vel_rad_s, stepRate, steps.position(),
                  steps.dropped(), gyroZ_filtered_rad_s);
  }
}
