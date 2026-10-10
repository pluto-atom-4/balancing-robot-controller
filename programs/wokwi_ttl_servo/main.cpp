#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#define DXL_TX_PIN 4
#define DXL_RX_PIN 5

#define DEMO_VEL 150
#define DUR_CW_MS 3000
#define DUR_STOP_MS 1000
#define DUR_CCW_MS 3000
#define DUR_TORQUE_OFF_MS 2000
#define DUR_TORQUE_ON_MS 2000
#define READ_PERIOD_MS 250
#define REPING_PERIOD_MS 2000

const float DXL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID = 1;

HardwareSerial dxl_serial(1);

class PinAwareSerialPort : public DYNAMIXEL::SerialPortHandler {
 public:
  PinAwareSerialPort(HardwareSerial& port, int dir_pin, int rx_pin, int tx_pin)
      : DYNAMIXEL::SerialPortHandler(port, dir_pin), port_(port), rx_pin_(rx_pin), tx_pin_(tx_pin) {}

  void begin() override {
    port_.begin(baud_, SERIAL_8N1, rx_pin_, tx_pin_);
    setOpenState(true);
  }

  void begin(unsigned long baud) override {
    baud_ = baud;
    begin();
  }

 private:
  HardwareSerial& port_;
  int rx_pin_, tx_pin_;
  unsigned long baud_ = 57600;
};

PinAwareSerialPort dxl_port(dxl_serial, -1, DXL_RX_PIN, DXL_TX_PIN);
Dynamixel2Arduino dxl;

struct Phase { const char* name; int32_t vel; int8_t torque; uint32_t dur_ms; };
static const int32_t NO_VEL = INT32_MIN;  // torque -1 = leave unchanged
static const Phase kPhases[] = {
  {"CW",         DEMO_VEL,  -1, DUR_CW_MS},
  {"STOP",       0,         -1, DUR_STOP_MS},
  {"CCW",        -DEMO_VEL, -1, DUR_CCW_MS},
  {"TORQUE_OFF", NO_VEL,     0, DUR_TORQUE_OFF_MS},
  {"TORQUE_ON",  NO_VEL,     1, DUR_TORQUE_ON_MS},
};
static const uint8_t PHASE_COUNT = sizeof(kPhases) / sizeof(kPhases[0]);

static bool servo_ok = false;
static uint8_t phase_idx = 0;
static uint32_t phase_start_ms = 0;
static uint32_t last_read_ms = 0;
static uint32_t last_ping_ms = 0;
static int32_t last_pos = 0;
static bool have_last = false;

static bool configure_servo() {
  bool w1 = dxl.torqueOff(DXL_ID);
  bool w2 = dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  bool w3 = dxl.torqueOn(DXL_ID);
  Serial.printf("[setup] torque_off=%d, mode=%d, torque_on=%d\n", w1, w2, w3);
  return w1 && w2 && w3;
}

static void enter_phase(uint8_t i) {
  const Phase& p = kPhases[i];
  bool ok = true;
  if (p.torque != -1) {
    bool r = p.torque ? dxl.torqueOn(DXL_ID) : dxl.torqueOff(DXL_ID);
    ok = ok && r;
  }
  if (p.vel != NO_VEL) {
    bool r = dxl.setGoalVelocity(DXL_ID, (float)p.vel);
    ok = ok && r;
  }
  const char* torque = p.torque == -1 ? "-" : (p.torque ? "1" : "0");
  if (p.vel == NO_VEL) {
    Serial.printf("[demo] phase=%s v=- torque=%s ok=%d\n", p.name, torque, ok);
  } else {
    Serial.printf("[demo] phase=%s v=%ld torque=%s ok=%d\n", p.name, (long)p.vel, torque, ok);
  }
  if (!ok) Serial.printf("[demo] WARN write failed phase=%s\n", p.name);

  phase_idx = i;
  phase_start_ms = millis();
  have_last = false;
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to settle

  dxl.setPort(dxl_port);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  Serial.println("[ESP32-C3] Initializing Dynamixel Target Bus...");

  // Ping the servo to make sure communication is established
  bool ping_ok = dxl.ping(DXL_ID);
  uint16_t err_code = dxl.getLastLibErrCode();
  Serial.printf("[ping] result=%d, error_code=%d", ping_ok, err_code);

  // Name the error code for debugging
  if (err_code == 0) Serial.println(" (OK)");
  else if (err_code == 1) Serial.println(" (PROCEEDING)");
  else if (err_code == 2) Serial.println(" (NOT_SUPPORTED)");
  else if (err_code == 3) Serial.println(" (TIMEOUT - no reply)");
  else if (err_code == 4) Serial.println(" (INVALID_ID)");
  else if (err_code == 5) Serial.println(" (NOT_SUPPORT_BROADCAST)");
  else if (err_code == 6) Serial.println(" (NULLPTR)");
  else Serial.printf(" (unknown #%d)\n", err_code);

  if (ping_ok) {
    Serial.println("[SUCCESS] Servo found online!");
  } else {
    Serial.println("[WARNING] Servo offline; proceeding anyway.");
  }

  servo_ok = ping_ok && configure_servo();
  last_ping_ms = millis();
  if (servo_ok) enter_phase(0);
  Serial.println("Reading live track data...");
}

// Expected on the simulated servo, one [pos] line per READ_PERIOD_MS:
//   CW delta about +586, STOP 0, CCW about -586,
//   TORQUE_OFF exactly 0 with identical pos on every read,
//   TORQUE_ON about -586 (stored CCW goal resumes).
// Velocity and torque are written only at phase changes (no keepalive).
void loop() {
  uint32_t now = millis();

  if (!servo_ok) {
    if (now - last_ping_ms >= REPING_PERIOD_MS) {
      last_ping_ms = now;
      if (dxl.ping(DXL_ID) && configure_servo()) {
        servo_ok = true;
        enter_phase(0);
      } else {
        Serial.println("[demo] waiting: servo offline");
      }
    }
    return;
  }

  if (now - phase_start_ms >= kPhases[phase_idx].dur_ms) {
    enter_phase((phase_idx + 1) % PHASE_COUNT);
  }

  if (now - last_read_ms >= READ_PERIOD_MS) {
    last_read_ms = now;
    int32_t pos = dxl.getPresentPosition(DXL_ID);
    const char* name = kPhases[phase_idx].name;
    if (dxl.getLastLibErrCode() == DXL_LIB_OK) {
      if (have_last) {
        int32_t delta = ((pos - last_pos + 6144) % 4096) - 2048;
        Serial.printf("[pos] phase=%s pos=%ld delta=%ld\n", name, (long)pos, (long)delta);
      } else {
        Serial.printf("[pos] phase=%s pos=%ld delta=n/a\n", name, (long)pos);
      }
      last_pos = pos;
      have_last = true;
    } else {
      Serial.printf("[pos] phase=%s read failed err=%d\n", name, dxl.getLastLibErrCode());
      have_last = false;
    }
  }
}
