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

// Segway simulation configuration (sim-only, unvalidated guesses).
static constexpr float    SIM_INIT_THETA_RAD = 0.262f; // Start at 15 degrees
static constexpr float    SIM_Kp = 2.0f;               // PD proportional gain (rad -> rad/s)
static constexpr float    SIM_Kd = 0.5f;               // PD derivative gain
static constexpr float    SIM_SERVO_CENTER = 90.0f;    // Servo center (0 wheel velocity)
static constexpr float    SIM_SERVO_RANGE = 90.0f;     // ±90 deg from center = ±wheel velocity max
static constexpr float    SIM_WHEEL_VEL_MAX = 2.0f;    // rad/s max wheel command
static constexpr float    SIM_IMU_DISTURBANCE_GAIN = 0.1f; // Scale IMU accel to disturbance (small)

// ===== Globals =====
Adafruit_MPU6050 mpu;
Servo myservo;
SegwaySim segway_sim(SIM_INIT_THETA_RAD);
uint32_t lastLoopMs = 0;
uint32_t lastPrintMs = 0;
float lastServoAngle_deg = SIM_SERVO_CENTER;

// ===== Helpers =====

// Map servo angle (0..180 deg) to wheel velocity command (rad/s).
// 90 deg = 0 rad/s (stopped)
// 0 deg = -SIM_WHEEL_VEL_MAX (full reverse)
// 180 deg = +SIM_WHEEL_VEL_MAX (full forward)
float servo_angle_to_wheel_vel(float angle_deg) {
  float normalized = (angle_deg - SIM_SERVO_CENTER) / SIM_SERVO_RANGE;
  normalized = constrain(normalized, -1.0f, 1.0f);
  return normalized * SIM_WHEEL_VEL_MAX;
}

// Clamp dt to reasonable range (handles jitter, prevents runaway).
float clamp_dt(uint32_t dt_ms) {
  float dt_sec = dt_ms / 1000.0f;
  if (dt_sec <= 0.0f) dt_sec = 0.02f;
  if (dt_sec > 0.1f) dt_sec = 0.1f;  // Cap at 100 ms
  return dt_sec;
}

// Convert accel magnitude to disturbance torque (rad/s^2).
// Wokwi slider pushes the MPU; we feed back as a frame disturbance.
float accel_to_disturbance(float accel_magnitude_g) {
  // Small coupling: disturbance = gain * accel.
  // (unvalidated, tuned for Wokwi visibility)
  return accel_magnitude_g * SIM_IMU_DISTURBANCE_GAIN;
}

// ===== Arduino entry points =====
void setup() {
  Serial.begin(115200);
  delay(1000); // Boot without blocking on USB host

  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("\n[ttl_servo] Segway Physics Simulator (Wokwi only, SIM-ONLY, UNVALIDATED)");
  Serial.println("[ttl_servo] Initializing MPU6050...");
  
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("[ttl_servo] Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  
  // Set gyro/accel range
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.println("[ttl_servo] MPU6050 found");

  // Initialize servo
  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(SERVO_FREQ_HZ);
  myservo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  myservo.write(lroundf(SIM_SERVO_CENTER)); // Start at center
  Serial.println("[ttl_servo] Servo initialized");
  
  // Init segway sim
  segway_sim.reset(SIM_INIT_THETA_RAD);
  Serial.println("[ttl_servo] Segway sim initialized at 15 deg");
  Serial.println("[ttl_servo] Ready. Use Wokwi slider to perturb.\n");

  lastLoopMs = millis();
  lastPrintMs = lastLoopMs;
}

void loop() {
  uint32_t now_ms = millis();
  uint32_t dt_ms = now_ms - lastLoopMs;

  // Run every LOOP_PERIOD_MS (50 Hz nominal)
  if (dt_ms < LOOP_PERIOD_MS) {
    delay(1);
    return;
  }
  lastLoopMs = now_ms;
  float dt_sec = clamp_dt(dt_ms);

  // ===== 1. READ SENSORS (MPU disturbance) =====
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  // Accel magnitude as disturbance (Wokwi slider effect).
  // Ignore gyro for sim (the sim's own gyro is the state theta_dot).
  float accel_mag = sqrtf(accel.acceleration.x * accel.acceleration.x +
                          accel.acceleration.y * accel.acceleration.y +
                          accel.acceleration.z * accel.acceleration.z);
  float disturbance_rad_s2 = accel_to_disturbance(accel_mag);

  // ===== 2. RUN PD CONTROL =====
  // Target: theta = 0 (upright)
  float error_theta = 0.0f - segway_sim.theta_rad;      // rad
  float error_rate = 0.0f - segway_sim.theta_dot_rad_s; // rad/s
  float control_output = SIM_Kp * error_theta + SIM_Kd * error_rate;
  
  // Map control output to servo angle (0..180 deg).
  // Assume control_output in units of rad/s command.
  float servo_target_deg = SIM_SERVO_CENTER + (control_output / SIM_WHEEL_VEL_MAX) * SIM_SERVO_RANGE;
  servo_target_deg = constrain(servo_target_deg, 0.0f, 180.0f);

  // ===== 3. WRITE SERVO =====
  int servo_cmd = lroundf(servo_target_deg);
  myservo.write(servo_cmd);
  lastServoAngle_deg = servo_target_deg;

  // ===== 4. MAP SERVO TO WHEEL VELOCITY =====
  float wheel_target_rad_s = servo_angle_to_wheel_vel(servo_target_deg);

  // ===== 5. STEP THE PHYSICS =====
  // Motor lag + inverted pendulum dynamics.
  segway_sim.step(wheel_target_rad_s, disturbance_rad_s2, dt_sec);

  // ===== 6. FALL DETECTION + RESET =====
  if (segway_sim.has_fallen) {
    Serial.println("[ttl_servo] FELL! Resetting sim.");
    segway_sim.reset(SIM_INIT_THETA_RAD);
  }

  // ===== 7. TELEMETRY (10 Hz) =====
  if (now_ms - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now_ms;
    Serial.printf("[ttl_servo] sim theta=%.3f rate=%.3f wheel=%.3f cmd=%d accel=%.2fg\n",
                  segway_sim.theta_rad,
                  segway_sim.theta_dot_rad_s,
                  segway_sim.wheel_vel_rad_s,
                  servo_cmd,
                  accel_mag);
  }
}
