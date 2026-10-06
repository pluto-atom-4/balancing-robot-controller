#pragma once

// Balance firmware constants. Arduino-free. This is the ONE place for servo baud, ids,
// wheel signs, servo pins, wheel speed cap and fail count (grep gate: these literals
// appear nowhere else). #32 EXTENDS this file below the servo block; do not recreate it.
// Nothing here is validated on hardware.

#include <cstdint>

// --- Servo bus (#31) ---
constexpr uint32_t kServoBaud = 57600;            // PROVEN (tilt_servo/main.cpp:34). The ONLY baud used.
constexpr uint32_t kServoBaudRequested = 2000000; // human-stated; UNTESTED on C3 + FE-URT-1. Switch kServoBaud to
                                                  // this ONLY after #44 shows 2M works AND the servos are set to 2M
                                                  // manually (Dynamixel Wizard). EEPROM writes are out of scope.
constexpr uint8_t kServoIdLeft = 1;               // TODO(human): ID 1 matches tilt_servo; confirm on hardware
constexpr uint8_t kServoIdRight = 2;              // TODO(human): ID 2 UNCONFIRMED
constexpr int8_t kLeftWheelSign = 1;              // TODO(human): which wheel is mirrored is UNCONFIRMED
constexpr int8_t kRightWheelSign = -1;            // TODO(human): mirrored mount UNCONFIRMED (#32 reuses this)
constexpr int kServoRxPin = 20;                   // D7, tilt_servo/main.cpp:36
constexpr int kServoTxPin = 21;                   // D6, tilt_servo/main.cpp:37
constexpr int kServoDirPin = -1;                  // FE-URT-1 handles direction in hardware (tilt_servo:35)
constexpr float kMaxWheelRadS = 1.0f;             // first bring-up cap, matches sim +-1.0 command limit
constexpr uint8_t kServoFailMax = 3;              // consecutive write failures before WheelPair latches
