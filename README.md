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

## Code Knowledge Graph

This repo indexes its own code with [code-review-graph](https://github.com/tirth8205/code-review-graph) (structural AST/call graph) and [graphify](https://pypi.org/project/graphifyy/) (macro architecture map), both wired into Claude Code via MCP and git hooks — see `CLAUDE.md` for the routing rules.

```bash
# Rebuild both graphs on a fresh clone
code-review-graph build
graphify update .

# Keep the code-review-graph graph live while you work
code-review-graph watch

# Query the structural graph directly
code-review-graph query callers_of <symbol>
code-review-graph impact --files <path>
code-review-graph detect-changes --brief

# Query the macro graph directly
graphify query "<question>"
graphify explain "<concept>"
```

Both graphs are gitignored (`.code-review-graph/`, `graphify-out/`) and rebuild locally per clone — the `pre-commit` and `PostToolUse`/`PreToolUse` hooks in `.claude/settings.json` keep them incrementally fresh while editing.
