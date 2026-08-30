# Balancing Robot Controller

Self-balancing robot firmware using PlatformIO, Seeed Studio XIAO S3 Sense, and Feetech STS3032 servos.

## Hardware

- **MCU**: Seeed Studio XIAO S3 Sense (ESP32-S3)
- **IMU**: Built-in 6-axis IMU (accelerometer + gyroscope)
- **Servos**: 2× Feetech STS3032 bus servos (servo protocol control)
- **Communication**: USB-C (programming), serial for servo control

## Development

Built with PlatformIO and developed in VS Code.

```bash
# Build firmware
pio run

# Upload to device
pio run -t upload

# Monitor serial output
pio device monitor

# Run tests
pio test
```

## Project Structure

- `src/main.cpp` – Main firmware entry point
- `include/` – Header files
- `lib/` – Custom libraries
- `platformio.ini` – Build and environment configuration
- `test/` – Unit tests

## Key Libraries

- PlatformIO frameworks and built-in esp32 support
- Custom servo control library for STS3032 bus protocol
- IMU sensor driver integration
