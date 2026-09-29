#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Dynamixel2Arduino.h>
#include <tilt.h>

// I2C SDA=D4 (GPIO6), SCL=D5 (GPIO7)
// Dynamixel bus TX=D6 (GPIO21), RX=D7 (GPIO20)
// Crossover: MCU TX -> FE-URT-1 RX, MCU RX -> FE-URT-1 TX
// Serial is USB CDC (build flags) so Serial1 is free for servo bus.

static constexpr uint32_t kI2cHz = 400000;
static constexpr uint32_t kLoopUs = 20000;    // 50 Hz
static constexpr uint32_t kPrintUs = 100000;  // 10 Hz
// +1: gyro Y > 0 raises pitch (see tilt::accelPitchDeg). Flip to -1 if fused moves opposite to accel on hardware.
static constexpr float GYRO_PITCH_SIGN = 1.0f;
static constexpr uint32_t kDispUs = 100000;   // 10 Hz OLED refresh (I2C push blocks ~25 ms)

// OLED config
static constexpr uint8_t kScreenW = 128;
static constexpr uint8_t kScreenH = 64;
static constexpr uint8_t kOledAddr = 0x3C;    // use 0x3D if strapped that way
static constexpr int kPlateW = 40;
static constexpr int kPlateH = 20;
static constexpr float kAccelRange = 10.0f;   // m/s^2 mapped to full width
static constexpr float kPitchRange = 45.0f;   // deg mapped to full height

// Dynamixel config
static constexpr uint8_t  DXL_ID = 1;
static constexpr float    DXL_PROTO = 2.0f;
static constexpr uint32_t DXL_BAUD = 57600;
static constexpr int      DXL_DIR_PIN = -1;   // FE-URT-1 handles direction in hardware
static constexpr int      DXL_RX = 20;        // D7
static constexpr int      DXL_TX = 21;        // D6
static constexpr uint16_t ADDR_PROFILE_ACCEL = 108;  // XL330, 4 bytes
static constexpr uint16_t ADDR_PROFILE_VEL = 112;    // XL330, 4 bytes
static constexpr uint32_t PROFILE_ACCEL = 20;        // UNVERIFIED units, conservative
static constexpr uint32_t PROFILE_VEL = 100;         // UNVERIFIED units, conservative

// First bring-up: +-30 deg about 2048 (341 ticks). NOT 1365..2731 (that is +-60 deg).
static constexpr float PITCH_LIMIT_DEG = 30.0f;
static constexpr int32_t SERVO_MIN_TICKS = 1707;
static constexpr int32_t SERVO_MAX_TICKS = 2389;
static constexpr float DEADBAND_DEG = 0.5f;
static constexpr int32_t MAX_TICKS_PER_LOOP = 8;     // ~35 deg/s slew at 50 Hz
static constexpr uint8_t kFailMax = 3;               // consecutive failures before failsafe

static Adafruit_MPU6050 mpu;
static tilt::ComplementaryFilter filt;
static uint32_t lastUs = 0;
static uint32_t lastPrintUs = 0;
static float accelDeg = 0.0f;
// clkAfter = kI2cHz so display() does not drop the shared bus to the 100 kHz default and slow MPU reads.
static Adafruit_SSD1306 display(kScreenW, kScreenH, &Wire, -1, kI2cHz, kI2cHz);
static bool displayOk = false;
static uint32_t lastDispUs = 0;

static Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
static bool torqueActive = false;   // true only after successful startup; latched off by failSafe()
static int32_t goalTicks = 2048;
static uint8_t imuFail = 0;
static uint8_t dxlFail = 0;

static void i2cScan() {
  uint8_t n = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { Serial.printf("[i2c] found 0x%02X\n", addr); ++n; }
  }
  if (n == 0) Serial.println("[i2c] none");
}

// Only used before torque is enabled (setup). Never call with torque on.
static void haltMsg(const char* msg) {
  for (;;) { Serial.println(msg); delay(2000); }
}

// Latched: torque off, stays off until reset. Loop keeps running read-only.
static void failSafe(const char* reason) {
  if (!torqueActive) return;
  torqueActive = false;
  bool off = false;
  for (uint8_t i = 0; i < 3 && !off; ++i) off = dxl.torqueOff(DXL_ID);
  Serial.printf("[safe] torque off %s%s\n", reason, off ? "" : " (TORQUE OFF NOT CONFIRMED)");
}

static float mapf(float x, float inLo, float inHi, float outLo, float outHi) {
  return outLo + (x - inLo) * (outHi - outLo) / (inHi - inLo);
}

static void drawPlate(float accelY, float pitchDeg) {
  float fx = mapf(accelY, -kAccelRange, kAccelRange, 0, kScreenW);
  float fy = mapf(pitchDeg, -kPitchRange, kPitchRange, 0, kScreenH);
  int cx = (int)tilt::clampf(fx, kPlateW/2, kScreenW-kPlateW/2);
  int cy = (int)tilt::clampf(fy, kPlateH/2, kScreenH-kPlateH/2);
  display.clearDisplay();
  display.drawRoundRect(cx-kPlateW/2, cy-kPlateH/2, kPlateW, kPlateH, 3, SSD1306_WHITE);
  display.drawPixel(cx, cy, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("P:%+.1f%s", pitchDeg, torqueActive ? "" : " T0");
  display.display();
}

// Returns true only if torque is on and goal == present position. On any failure torque is left off.
static bool servoStart() {
  // Order matters: begin() first (may re-init port), then Serial1.begin with explicit pins last.
  dxl.begin(DXL_BAUD);
  Serial1.begin(DXL_BAUD, SERIAL_8N1, DXL_RX, DXL_TX);
  dxl.setPortProtocolVersion(DXL_PROTO);

  if (!dxl.ping(DXL_ID)) {
    Serial.println("[dxl] ping failed");
    return false;
  }
  Serial.println("[dxl] ping ok");

  if (!dxl.torqueOff(DXL_ID) ||
      !dxl.setOperatingMode(DXL_ID, OP_POSITION)) {
    Serial.println("[dxl] config failed");
    return false;
  }
  uint32_t acc = PROFILE_ACCEL, vel = PROFILE_VEL;
  if (!dxl.write(DXL_ID, ADDR_PROFILE_ACCEL, reinterpret_cast<uint8_t*>(&acc), 4, 100) ||
      !dxl.write(DXL_ID, ADDR_PROFILE_VEL, reinterpret_cast<uint8_t*>(&vel), 4, 100)) {
    Serial.println("[dxl] profile write failed");
    return false;
  }
  // Start from current position so enabling torque does not jump.
  const float present = dxl.getPresentPosition(DXL_ID);
  if (dxl.getLastLibErrCode() != 0) {
    Serial.println("[dxl] read present position failed");
    return false;
  }
  goalTicks = static_cast<int32_t>(tilt::clampf(present,
                  static_cast<float>(SERVO_MIN_TICKS), static_cast<float>(SERVO_MAX_TICKS)));
  if (!dxl.setGoalPosition(DXL_ID, goalTicks)) {
    Serial.println("[dxl] set initial goal failed");
    return false;
  }
  if (!dxl.torqueOn(DXL_ID)) {
    dxl.torqueOff(DXL_ID);
    Serial.println("[dxl] torque on failed");
    return false;
  }
  Serial.printf("[dxl] torque on, start goal=%ld\n", static_cast<long>(goalTicks));
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[tilt] boot");
  Wire.begin(D4, D5);
  Wire.setClock(kI2cHz);
  if (!mpu.begin(0x68, &Wire) && !mpu.begin(0x69, &Wire)) {
    Serial.println("[imu] MPU6050 not found");
    i2cScan();
    haltMsg("[imu] halted: check wiring (SDA=D4 SCL=D5, AD0)");  // torque never enabled yet: safe
  }
  Wire.setClock(kI2cHz);  // in case begin() reset the bus clock
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // Display is optional: never halt on failure. periphBegin=false: Wire already started on D4/D5.
  displayOk = display.begin(SSD1306_SWITCHCAPVCC, kOledAddr, true, false);
  if (displayOk) {
    display.clearDisplay();
    display.display();
    Serial.println("[oled] ready");
  } else {
    Serial.println("[oled] init failed");
  }
  Wire.setClock(kI2cHz);
  delay(100);
  Serial.println("[imu] ready");

  // Servo last, after IMU proven. On failure torque stays off and IMU/OLED run read-only.
  torqueActive = servoStart();
  if (!torqueActive) Serial.println("[dxl] read-only mode (torque off)");

  lastUs = micros();
  lastPrintUs = lastUs;
  lastDispUs = lastUs;
}

void loop() {
  const uint32_t now = micros();
  const uint32_t elapsed = now - lastUs;  // unsigned: rollover-safe
  if (elapsed < kLoopUs) return;
  lastUs = now;
  const float dt = static_cast<float>(elapsed) * 1e-6f;

  sensors_event_t a, g, t;
  if (!mpu.getEvent(&a, &g, &t)) {
    if (++imuFail >= kFailMax) failSafe("imu read failed");
    return;
  }
  imuFail = 0;
  if (!isfinite(a.acceleration.x) || !isfinite(a.acceleration.y) || !isfinite(a.acceleration.z) ||
      !isfinite(g.gyro.y)) {
    failSafe("imu nan");
    return;  // do not feed NaN into filter
  }

  accelDeg = tilt::accelPitchDeg(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  const float gyroDegPerSec = GYRO_PITCH_SIGN * g.gyro.y * tilt::kRadToDeg;
  const float fused = filt.update(accelDeg, gyroDegPerSec, dt);
  if (!isfinite(fused)) {
    failSafe("pitch nan");
    return;
  }

  if (torqueActive) {
    const float pitch = tilt::clampf(fused, -PITCH_LIMIT_DEG, PITCH_LIMIT_DEG);
    int32_t target = tilt::degToTicks(pitch, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
    // Deadband in ticks derived from DEADBAND_DEG (11.38 ticks/deg, rounded, not truncated).
    const int32_t deadTicks = static_cast<int32_t>(lroundf(DEADBAND_DEG * (4096.0f / 360.0f)));
    int32_t diff = target - goalTicks;
    if (diff <= deadTicks && diff >= -deadTicks) diff = 0;
    if (diff > MAX_TICKS_PER_LOOP) diff = MAX_TICKS_PER_LOOP;
    if (diff < -MAX_TICKS_PER_LOOP) diff = -MAX_TICKS_PER_LOOP;
    const int32_t next = goalTicks + diff;
    if (dxl.setGoalPosition(DXL_ID, next)) {
      goalTicks = next;
      dxlFail = 0;
    } else if (++dxlFail >= kFailMax) {
      failSafe("dxl lost");
    }
  }

  if (now - lastPrintUs >= kPrintUs) {
    lastPrintUs = now;
    Serial.printf("[imu] accel=%.2f fused=%.2f goal=%ld torque=%d\n",
                  accelDeg, fused, static_cast<long>(goalTicks), torqueActive ? 1 : 0);
  }

  if (displayOk && now - lastDispUs >= kDispUs) {
    lastDispUs = now;
    drawPlate(a.acceleration.y, fused);
  }
}
