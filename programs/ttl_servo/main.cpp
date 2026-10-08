// ttl_servo: closed-loop Segway physics simulation for Wokwi.
// SIM-ONLY, UNVERIFIED on hardware. The firmware simulates the Segway frame
// (lib/segway_sim), balances it with a PD loop, and writes the result to a
// continuous-rotation servo (90 deg = stop, above = forward, below = reverse).
// The real MPU6050 gyro Z is a disturbance rate injected into the simulated
// tilt dynamics (move the Wokwi gyro Z slider to push the frame).
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <math.h>
#include <segway_sim.h>

// ===== Configuration =====
static constexpr uint8_t  SERVO_PIN = 10;              // GPIO10, matches diagram.json wiring
static constexpr uint8_t  I2C_SDA = 8;
static constexpr uint8_t  I2C_SCL = 9;
static constexpr uint8_t  MPU_ADDR = 0x68;
static constexpr int16_t  SERVO_MIN_US = 500;
static constexpr int16_t  SERVO_MAX_US = 2400;
static constexpr uint16_t SERVO_FREQ_HZ = 50;
static constexpr uint32_t LOOP_PERIOD_MS = 20;         // 50 Hz main loop
static constexpr uint32_t PRINT_PERIOD_MS = 100;       // 10 Hz serial output
static constexpr float    SIM_INIT_THETA_RAD = 0.262f; // start leaning 15 deg
static constexpr float    GYRO_FILTER_TAU_MS = 50.0f;  // gyro Z low-pass
static constexpr int      GYRO_CAL_SAMPLES = 100;      // ~1 s at boot, sim at rest
static constexpr int      SERVO_STOP_DEG = 90;

// ===== Globals =====
Adafruit_MPU6050 mpu;
Servo myservo;
segway::SegwaySim segway_sim(SIM_INIT_THETA_RAD);
uint32_t lastLoopMs = 0;
uint32_t lastPrintMs = 0;
float gyroZ_bias_rad_s = 0.0f;
float gyroZ_filtered_rad_s = 0.0f;
int lastServoDeg = SERVO_STOP_DEG;

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

// ===== Arduino entry points =====
void setup() {
  Serial.begin(115200);
  delay(1000); // Boot without blocking on USB host

  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("\n[ttl_servo] Segway physics sim (Wokwi, SIM-ONLY, UNVERIFIED)");

  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("[ttl_servo] Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Servo first: hold it at stop before the (blocking) calibration.
  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(SERVO_FREQ_HZ);
  myservo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  myservo.write(SERVO_STOP_DEG);

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

  segway_sim.reset(SIM_INIT_THETA_RAD);
  Serial.println("[ttl_servo] Sim starts at 15 deg; PD should settle to 0. Move gyro Z to push.\n");

  lastLoopMs = millis();
  lastPrintMs = lastLoopMs;
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

  // 2. PD -> servo -> physics, same code the native tests exercise.
  int servoDeg = segway_sim.tick(gyroZ_filtered_rad_s, clamp_dt_s(dt_ms));

  // 3. Drive the continuous-rotation servo (write only on change).
  if (servoDeg != lastServoDeg) {
    myservo.write(servoDeg);
    lastServoDeg = servoDeg;
  }

  // 4. Fall: stop the wheel, report, restart the sim.
  if (segway_sim.has_fallen) {
    myservo.write(SERVO_STOP_DEG);
    lastServoDeg = SERVO_STOP_DEG;
    Serial.printf("[ttl_servo] FELL theta=%.3f, reset\n", segway_sim.theta_rad);
    segway_sim.reset(SIM_INIT_THETA_RAD);
  }

  // 5. Telemetry (10 Hz)
  if (now_ms - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now_ms;
    Serial.printf("[ttl_servo] sim theta=%.3f rate=%.3f wheel=%.2f cmd=%d gyroZ=%.3f\n",
                  segway_sim.theta_rad, segway_sim.theta_dot_rad_s,
                  segway_sim.wheel_vel_rad_s, servoDeg, gyroZ_filtered_rad_s);
  }
}
