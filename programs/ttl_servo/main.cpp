#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <math.h>

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
static constexpr float    FILTER_TAU_MS = 100.0f;      // Low-pass filter time constant (ms)
static constexpr float    DEADBAND_DPS = 2.0f;         // Deadband around 0 deg/s
static constexpr float    MAX_SLEW_DEG_PER_TICK = 6.0f; // Max servo step per tick (deg)
static constexpr float    GYRO_FULL_SCALE_DPS = 250.0f; // ±250 deg/s range
static constexpr float    CENTER_ANGLE_DEG = 90.0f;    // Servo neutral (0 deg/s maps here)

// ===== Globals =====
Adafruit_MPU6050 mpu;
Servo myservo;
uint32_t lastLoopMs = 0;
uint32_t lastPrintMs = 0;
float gyroZ_filtered_dps = 0.0f;    // Filtered gyro Z in deg/s
float lastServoAngle_deg = CENTER_ANGLE_DEG;  // Last written angle

// ===== Helpers =====
// Exponential moving average (EMA) filter
float ema_update(float filtered_prev, float raw_new, uint32_t dt_ms) {
  if (dt_ms == 0) return filtered_prev;
  float alpha = (float)dt_ms / (FILTER_TAU_MS + dt_ms);
  return filtered_prev * (1.0f - alpha) + raw_new * alpha;
}

// Apply deadband: small rates become exactly 0
float apply_deadband(float rate_dps) {
  if (fabsf(rate_dps) < DEADBAND_DPS) return 0.0f;
  return rate_dps;
}

// Linear map: rate (dps) -> servo angle (0..180 deg)
// 0 deg/s -> 90 deg (center)
// -250 deg/s -> 0 deg (min)
// +250 deg/s -> 180 deg (max)
float rate_to_angle(float rate_dps) {
  // Clamp to the full scale range
  rate_dps = constrain(rate_dps, -GYRO_FULL_SCALE_DPS, GYRO_FULL_SCALE_DPS);
  // Linear map: -250..+250 dps -> 0..180 deg
  return CENTER_ANGLE_DEG + (rate_dps / GYRO_FULL_SCALE_DPS) * 90.0f;
}

// Slew limiter: limit step size to avoid servo jerk
float apply_slew_limit(float target_angle, float last_angle, float max_step) {
  float delta = target_angle - last_angle;
  if (delta > max_step) return last_angle + max_step;
  if (delta < -max_step) return last_angle - max_step;
  return target_angle;
}

// ===== Arduino entry points =====
void setup() {
  Serial.begin(115200);
  delay(1000); // Boot without blocking on USB host

  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("\n[ttl_servo] Starting...");
  Serial.println("[ttl_servo] Initializing MPU6050...");
  
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("[ttl_servo] Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  
  // Set gyro range explicitly to ±250 deg/s
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.println("[ttl_servo] MPU6050 found, gyro range set to ±250 deg/s");

  // Initialize servo
  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(SERVO_FREQ_HZ);
  myservo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  myservo.write(lroundf(CENTER_ANGLE_DEG)); // Start at center
  Serial.println("[ttl_servo] Servo initialized at GPIO10");
  Serial.println("[ttl_servo] Ready. Waiting for gyro input...\n");

  lastLoopMs = millis();
  lastPrintMs = lastLoopMs;
}

void loop() {
  uint32_t now_ms = millis();
  uint32_t dt_ms = now_ms - lastLoopMs;

  // Run every LOOP_PERIOD_MS (50 Hz nominal, but dt varies)
  if (dt_ms < LOOP_PERIOD_MS) {
    delay(1); // Yield to avoid busy-wait
    return;
  }
  lastLoopMs = now_ms;

  // Read IMU
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Extract Z-axis gyro (radians/sec) and convert to degrees/sec
  float gyroZ_raw_dps = gyro.gyro.z * 57.2958f; // RAD_TO_DEG approximation

  // Apply EMA low-pass filter
  gyroZ_filtered_dps = ema_update(gyroZ_filtered_dps, gyroZ_raw_dps, dt_ms);

  // Apply deadband
  float gyroZ_deadband_dps = apply_deadband(gyroZ_filtered_dps);

  // Map to servo angle (0..180 deg)
  float target_angle_deg = rate_to_angle(gyroZ_deadband_dps);

  // Apply slew limiter
  float final_angle_deg = apply_slew_limit(target_angle_deg, lastServoAngle_deg, MAX_SLEW_DEG_PER_TICK);

  // Write to servo only if changed (to reduce jitter and wear)
  int final_angle_int = lroundf(final_angle_deg);
  int last_angle_int = lroundf(lastServoAngle_deg);
  if (final_angle_int != last_angle_int) {
    myservo.write(final_angle_int);
    lastServoAngle_deg = final_angle_deg;
  }

  // Periodic diagnostic output (10 Hz)
  if (now_ms - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now_ms;
    Serial.printf("[ttl_servo] raw=%.1f deadband=%.1f filt=%.1f target=%.1f final=%d\n",
                  gyroZ_raw_dps, gyroZ_deadband_dps, gyroZ_filtered_dps, target_angle_deg, final_angle_int);
  }
}
