#include <Arduino.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

static bool ledOn = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
#if !defined(__AVR_ATtiny85__)
  Serial.begin(115200);
#endif
}

void loop() {
  ledOn = !ledOn;
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
#if !defined(__AVR_ATtiny85__)
  Serial.println("[blink] tick");
#endif
  delay(3000);
}
