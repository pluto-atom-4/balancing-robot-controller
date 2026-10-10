#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#define DXL_TX_PIN 4
#define DXL_RX_PIN 5

const float DXL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID = 1;
const uint16_t DXL_MODEL_NUMBER = 1200; // XL330_M288

const uint16_t ADDR_OPERATING_MODE = 11;
const uint16_t ADDR_TORQUE_ENABLE  = 64;
const uint16_t ADDR_GOAL_VELOCITY  = 104;

const uint8_t OPERATING_MODE_VELOCITY = 1;

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

static bool writeReg8(uint16_t addr, uint8_t v) {
  return dxl.write(DXL_ID, addr, &v, 1, 100);
}

static bool writeReg32(uint16_t addr, int32_t v) {
  uint8_t b[4] = {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)};
  return dxl.write(DXL_ID, addr, b, 4, 100);
}

#ifdef WOKWI_TTL_SERVO_RAWDIAG
static void rawPingDiag() {
  const uint8_t ping_frame[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E};
  Serial.print("[diag] Sending raw PING: ");
  for (size_t i = 0; i < sizeof(ping_frame); i++) {
    Serial.printf("%02X ", ping_frame[i]);
  }
  Serial.println();
  
  dxl_serial.write(ping_frame, sizeof(ping_frame));
  delay(20);
  
  Serial.print("[diag] RX buffer: ");
  size_t count = 0;
  while (dxl_serial.available() && count < 32) {
    uint8_t b = dxl_serial.read();
    Serial.printf("%02X ", b);
    count++;
  }
  Serial.println();
  Serial.printf("[diag] RX count: %zu\n", count);
}
#endif

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to settle

  dxl.setPort(dxl_port);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  Serial.println("[ESP32-C3] Initializing Dynamixel Target Bus...");

#ifdef WOKWI_TTL_SERVO_RAWDIAG
  rawPingDiag();
#endif

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
  
  if (!ping_ok) {
    Serial.println("[info] Servo did not respond to PING; attempting to set model number...");
    dxl.setModelNumber(DXL_ID, DXL_MODEL_NUMBER);
  }

  if (ping_ok) {
    Serial.println("[SUCCESS] Servo found online!");
  } else {
    Serial.println("[WARNING] Servo offline; proceeding with manual register writes.");
  }

  // Set up velocity mode configurations
  bool w1 = writeReg8(ADDR_TORQUE_ENABLE, 0);
  bool w2 = writeReg8(ADDR_OPERATING_MODE, OPERATING_MODE_VELOCITY);
  bool w3 = writeReg8(ADDR_TORQUE_ENABLE, 1);
  Serial.printf("[setup] torque_off=%d, mode=%d, torque_on=%d\n", w1, w2, w3);

  // Command continuous forward rotation
  int32_t active_speed = 150;
  bool w4 = writeReg32(ADDR_GOAL_VELOCITY, active_speed);
  Serial.printf("[setup] goal_velocity=%d, result=%d\n", active_speed, w4);
  Serial.println("Reading live track data...");
}

void loop() {
  // Query the simulation part for its live encoder value
  int32_t present_position = dxl.getPresentPosition(DXL_ID);

  if (dxl.getLastLibErrCode() == DXL_LIB_OK) {
    Serial.print("Servo Horn Position (0-4095): ");
    Serial.println(present_position);
  } else {
    Serial.print("Read Failed! Error Code: ");
    Serial.println(dxl.getLastLibErrCode());
  }

  delay(250); // Refresh telemetric reading data 4 times a second
}
