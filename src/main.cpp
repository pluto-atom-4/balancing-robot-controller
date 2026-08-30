#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Balancing Robot Controller initialized");
}

void loop() {
  delay(100);
}
