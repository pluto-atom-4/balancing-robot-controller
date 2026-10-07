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

// --- Loop and safety (#32). All UNVERIFIED on hardware. ---
constexpr uint32_t kUsbSerialBaud = 115200;     // USB CDC monitor
constexpr uint32_t kI2cHz = 400000;             // as tilt_servo
// 50 Hz. Nominal 20 ms per freecad-workspace#345 (real Webots pattern 16,16,16,32 ms, mean 20 ms).
constexpr uint32_t kLoopUs = 20000;
// #345 gives: sustained MEAN period <= 21.5 ms, iid jitter +-9 ms, <= 3 consecutive missed updates.
// That is a linearized LQR SIM model, NOT a C3 measurement (PID untested, no compute delay modelled).
// loop_stats compares ONE period to this, so it is derived as nominal + 9 ms = 29 ms (the mean limit is 21.5 ms).
// TODO(human): confirm this derivation; #43 c3_probe gives the real numbers. R3 stays open.
constexpr uint32_t kJitterBudgetUs = 29000;
// Latch if a Running tick gap exceeds this. Must exceed 4*kLoopUs (#345 tolerates 3 missed updates = 80 ms gap).
// Only detects AFTER the stall (on resume); a true hang cannot be handled in software. TODO(human): confirm.
constexpr uint32_t kStallUs = 100000;
constexpr float kTiltCutoffRad = 0.5f;          // |pitch - center| above this: torque off, latched until reset
// Motors stay off unless |pitch - center| <= this for kLeanGateHoldTicks ticks. TODO(human): the value is
// a SIM analogue (#345 uses theta0 = 0.05 rad, bound 2*theta0 = 0.1), NOT a hardware number.
constexpr float kStartupLeanGateRad = 0.1f;
constexpr uint8_t kLeanGateHoldTicks = 25;      // 0.5 s at 50 Hz. TODO(human): confirm.
constexpr uint8_t kImuFailMax = 3;              // consecutive IMU read failures before latch
// 1:1 rad/s, shared by LQR and PID. LQR and PID give OPPOSITE signs for the same tilt (UNVERIFIED,
// freecad-workspace#359); HIL #34 steps 5 and 6 check both. Tune in HIL.
constexpr float kWheelCmdScale = 1.0f;
// false = compute and log only, never write wheels (one-line dry run). Default true per #32.
constexpr bool kWheelWritesEnabled = true;
// TODO(human): optional BOOT button (GPIO9, strapping pin) as hardware kill is NOT implemented.
