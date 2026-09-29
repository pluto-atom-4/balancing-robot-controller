#include <Arduino.h>

// I2C SDA=D4 (GPIO6), SCL=D5 (GPIO7)
// Dynamixel bus TX=D6 (GPIO21), RX=D7 (GPIO20)
// Crossover: MCU TX -> FE-URT-1 RX, MCU RX -> FE-URT-1 TX
// Serial is USB CDC (build flags) so Serial1 is free for servo bus.

void setup() {
  Serial.begin(115200);
  Serial.println("[tilt] boot");
}

void loop() {
  delay(1000);
}
