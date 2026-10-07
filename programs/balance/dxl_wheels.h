#pragma once

// Dynamixel XL330 driver for wheel_servo::IWheelServo (#31). Arduino-bound.
// Include ONLY from programs/balance/main.cpp (built by the balance-xiao_c3* envs, which provide
// Dynamixel2Arduino; no other env builds the balance program).
// Proven on the C3 only: init order, protocol 2.0, DIR -1, raw dxl.write (tilt_servo).
// Raw dxl.read and the register addresses below are UNVERIFIED on hardware.
// Addresses: D2A actuator.cpp XL330 table; 44/104/128 also cited in lib/dxl_units/dxl_units.h
// (e-Manual). 11 (operating mode) and 64 (torque enable) are NOT yet cited: UNVERIFIED vs e-Manual.
// Raw read/write is used (not readControlTableItem): D2A returns 0 on failure with a sticky
// error code, and its defaults use 100 ms timeouts.
// Never writes Velocity Limit. Writes Operating Mode (EEPROM area) only if it is not already velocity.

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <cstdint>
#include <wheel_servo.h>
#include "config.h"

constexpr float kDxlProtocol = 2.0f;
constexpr uint32_t kDxlIoTimeoutMs = 20;      // UNVERIFIED; tune from #43/#44 (acked 4-byte write is ~5 ms at the bring-up baud)
constexpr uint16_t kDxlAddrOperatingMode = 11;
constexpr uint16_t kDxlAddrVelocityLimit = 44;
constexpr uint16_t kDxlAddrTorqueEnable = 64;
constexpr uint16_t kDxlAddrGoalVelocity = 104;
constexpr uint16_t kDxlAddrPresentVelocity = 128;
constexpr uint8_t kDxlOpModeVelocity = static_cast<uint8_t>(OP_VELOCITY);  // 1

class DxlServo : public wheel_servo::IWheelServo {
 public:
  DxlServo() : dxl_(Serial1, kServoDirPin) {}

  // Init order proven in tilt_servo: dxl.begin -> Serial1.begin(explicit pins) -> protocol.
  // D2A begin() returns void, so success here means "init sequence ran"; ping() is the real link check.
  bool begin(uint32_t baud) override {
    fault_ = wheel_servo::Fault::None;
    dxl_.begin(baud);
    Serial1.begin(baud, SERIAL_8N1, kServoRxPin, kServoTxPin);
    dxl_.setPortProtocolVersion(kDxlProtocol);
    return true;
  }

  bool ping(uint8_t id) override {
    if (dxl_.ping(id)) return true;
    fault_ = wheel_servo::Fault::NoPing;
    return false;
  }

  // false = communication failure. true with have_vel_limit=false = limit unreadable or <= 0.
  bool readInfo(uint8_t id, wheel_servo::DeviceInfo& out) override {
    out.model_number = 0;
    out.have_vel_limit = false;
    out.vel_limit_raw = 0;
    const uint16_t model = dxl_.getModelNumber(id);  // 0xFFFF = read failed
    if (model == 0xFFFF) {
      fault_ = wheel_servo::Fault::ReadFail;
      return false;
    }
    out.model_number = model;
    int32_t limit = 0;
    if (readReg(id, kDxlAddrVelocityLimit, 4, limit) && limit > 0) {
      out.have_vel_limit = true;
      out.vel_limit_raw = limit;
    }
    return true;
  }

  // 3 tries; confirmed only when the torque-enable register reads back 0.
  bool torqueOff(uint8_t id) override {
    for (uint8_t i = 0; i < 3; ++i) {
      (void)writeReg(id, kDxlAddrTorqueEnable, 1, 0);
      int32_t t = 1;
      if (readReg(id, kDxlAddrTorqueEnable, 1, t) && t == 0) return true;
    }
    fault_ = wheel_servo::Fault::TorqueOffUnconfirmed;
    return false;
  }

  // torque off -> (read mode; write ONLY if not velocity) -> goal 0 acked -> torque on read back == 1.
  // Any failure: torque off again and return false.
  bool enableVelocityMode(uint8_t id) override {
    if (!torqueOff(id)) return false;
    int32_t mode = -1;
    if (!readReg(id, kDxlAddrOperatingMode, 1, mode)) return failOff(id, wheel_servo::Fault::ReadFail);
    if (mode != kDxlOpModeVelocity) {
      if (!dxl_.setOperatingMode(id, OP_VELOCITY)) return failOff(id, wheel_servo::Fault::WriteFail);
    }
    if (!writeReg(id, kDxlAddrGoalVelocity, 4, 0)) return failOff(id, wheel_servo::Fault::WriteFail);
    if (!writeReg(id, kDxlAddrTorqueEnable, 1, 1)) return failOff(id, wheel_servo::Fault::WriteFail);
    int32_t t = 0;
    if (!readReg(id, kDxlAddrTorqueEnable, 1, t) || t != 1) return failOff(id, wheel_servo::Fault::WriteFail);
    return true;
  }

  bool setVelocityRaw(uint8_t id, int32_t raw) override {
    if (writeReg(id, kDxlAddrGoalVelocity, 4, raw)) return true;
    fault_ = wheel_servo::Fault::WriteFail;
    return false;
  }

  bool presentVelocityRaw(uint8_t id, int32_t& out) override {
    int32_t v = 0;
    if (!readReg(id, kDxlAddrPresentVelocity, 4, v)) {
      fault_ = wheel_servo::Fault::ReadFail;
      return false;
    }
    out = v;
    return true;
  }

  const dxl_units::VelocityUnits& units() const override { return dxl_units::kXl330; }
  const char* family() const override { return "XL330"; }
  wheel_servo::Fault lastFault() const override { return fault_; }

  // Read-only helpers for probe c3_facts (#44). NOT on IWheelServo (family-specific). false = comm failure.
  bool readOperatingMode(uint8_t id, int32_t& mode) { return readReg(id, kDxlAddrOperatingMode, 1, mode); }
  bool readTorqueEnable(uint8_t id, int32_t& on) { return readReg(id, kDxlAddrTorqueEnable, 1, on); }

 private:
  // Little-endian target (RISC-V C3, XL330): low bytes of the int32 hold 1/2/4 byte registers.
  bool readReg(uint8_t id, uint16_t addr, uint16_t len, int32_t& out) {
    int32_t v = 0;
    const int32_t n = dxl_.read(id, addr, len, reinterpret_cast<uint8_t*>(&v), sizeof(v), kDxlIoTimeoutMs);
    if (n != static_cast<int32_t>(len)) return false;
    out = v;
    return true;
  }
  bool writeReg(uint8_t id, uint16_t addr, uint16_t len, int32_t value) {
    return dxl_.write(id, addr, reinterpret_cast<const uint8_t*>(&value), len, kDxlIoTimeoutMs);
  }
  bool failOff(uint8_t id, wheel_servo::Fault f) {
    (void)torqueOff(id);   // best effort; sets fault_ to TorqueOffUnconfirmed if it cannot confirm
    if (fault_ != wheel_servo::Fault::TorqueOffUnconfirmed) fault_ = f;
    return false;
  }

  Dynamixel2Arduino dxl_;
  wheel_servo::Fault fault_ = wheel_servo::Fault::None;
};
