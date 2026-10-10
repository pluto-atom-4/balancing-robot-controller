#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#define DXL_TX_PIN 4
#define DXL_RX_PIN 5

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

  // Set up velocity mode configurations
  bool w1 = dxl.torqueOff(DXL_ID);
  bool w2 = dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  bool w3 = dxl.torqueOn(DXL_ID);
  Serial.printf("[setup] torque_off=%d, mode=%d, torque_on=%d\n", w1, w2, w3);

  // Command continuous forward rotation
  int32_t active_speed = 150;
  bool w4 = dxl.setGoalVelocity(DXL_ID, active_speed);
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
