# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Self-balancing robot firmware using PlatformIO + ESP32-S3. Hardware: Seeed Studio XIAO S3 Sense MCU, built-in 6-axis IMU, two Feetech STS3032 bus servos controlled via serial protocol.

## Development Commands

**Build firmware**
```bash
pio run
```

**Upload to XIAO S3**
```bash
pio run -t upload
```

**Monitor serial output** (115200 baud)
```bash
pio device monitor
```

**Run unit tests**
```bash
pio test
```

**Clean build**
```bash
pio run -t clean
```

## Architecture

### Core Layers

1. **Servo Control Layer** (`lib/servo/`)
   - STS3032 bus servo protocol implementation
   - Position/speed/torque commands
   - Multi-servo coordination (left/right balance)

2. **IMU/Sensor Layer** (`lib/imu/`)
   - XIAO S3 built-in accelerometer & gyroscope
   - Real-time pitch/roll angle estimation
   - Sensor fusion if needed

3. **Control Algorithm** (`src/controller/`)
   - PID loops for balance (tilt stabilization)
   - Motor speed regulation
   - Feedback integration

4. **Main Loop** (`src/main.cpp`)
   - Read IMU → compute error → send servo commands
   - Runs at ~100Hz (typical for balance control)

### File Structure

- `src/main.cpp` – Entry point, main control loop
- `include/` – Public headers
- `lib/` – Reusable libraries (servo driver, sensor wrappers)
- `test/` – Unit tests
- `platformio.ini` – Build config, board: `seeed_xiao_esp32s3`, framework: Arduino

## Key Constants/Configs

- **Serial Baud**: 115200 (monitoring & servo communication)
- **Target Loop Freq**: ~100Hz (10ms per cycle)
- **Servo ID Range**: 1–2 (left/right motors)
- **STS3032 Limits**: Position 0–4095 (270°), Speed 1–2047 units

## Build Notes

- Board: `seeed_xiao_esp32s3` (espressif32 platform)
- Upload speed: 921600 baud (high-speed upload)
- C++17 enabled
- No external dependencies yet (add as needed)

## Common Edits

- **Tune PID gains**: Look for `Kp`, `Ki`, `Kd` constants in controller code
- **Change balance setpoint**: Modify target angle in main loop
- **Add sensor filtering**: IMU data is noisy; consider complementary/Kalman filter
- **Servo command mapping**: Servo protocol commands in `lib/servo/STS3032.cpp`

## Testing

Unit tests in `test/`. Run with `pio test`. Native platform used for desktop testing of control algorithms.

## Serial Debugging

Servo commands and IMU readings logged to Serial. Use `pio device monitor` to inspect. Format: `[tag] message` for structured output.
