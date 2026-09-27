# Balancing Robot Controller

Multi-program PlatformIO project for several dev boards (Arduino Uno R3, Arduino UNO R4 WiFi, Seeed XIAO ESP32S3, generic ESP32, ATtiny85). Its main program is `balance`, self-balancing robot firmware for the Seeed Studio XIAO S3 Sense with Feetech STS3032 servos. Each program lives in `programs/<name>/` and shares code from `lib/`.

## Hardware (balance program)

- **MCU**: Seeed Studio XIAO S3 Sense (ESP32-S3)
- **IMU**: Built-in 6-axis IMU (accelerometer + gyroscope)
- **Servos**: 2× Feetech STS3032 bus servos (servo protocol control)
- **Communication**: USB-C (programming), serial for servo control

### Supported boards

- Seeed XIAO ESP32S3 (`xiao_s3`, balance + blink)
- Generic ESP32 dev board (`esp32dev`, balance + blink)
- Arduino Uno R3 ATmega328P (`uno`, blink only)
- Arduino UNO R4 WiFi (`uno_r4_wifi`, blink only)
- ATtiny85 (`attiny85`, blink only; ISP programmer, no serial)

## Python environment (PlatformIO)

This project requires PlatformIO Core (`pio`) on your PATH. All build settings live in `platformio.ini`, so any PlatformIO Core installation works. You have two options to get `pio`:

**Option A: user-level PlatformIO (use if already installed)**

If PlatformIO is already installed via the VS Code extension or the PlatformIO installer script, it lives in `~/.platformio/penv` (its own Python environment). It works for this project as-is, no project setup needed. Verified: PlatformIO Core 6.2.0 (Python 3.13) from `~/.platformio/penv` built this project's `blink-uno_r4_wifi` env.

To use it, check that it is available:
```bash
~/.platformio/penv/bin/pio --version
```
or `pio --version` if already on PATH. Then make it available in a terminal with one of:
- Activate: `source ~/.platformio/penv/bin/activate`
- Add to PATH in your shell profile: `export PATH="$HOME/.platformio/penv/bin:$PATH"`
- Do nothing for `make`: when `pio` is not on PATH, the Makefile falls back to `~/.platformio/penv/bin/pio` automatically, so `make build` works without any setup. Override with `make PIO=<path-to-pio> build`.

Downside: shared across all projects; upgrading it affects them all.

**Option B: project-level virtual environment (isolated, reproducible)**

For a clean setup or to pin a specific version, create an isolated Python environment in `.venv/`:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip platformio
pio --version
```

Every new terminal, activate the environment:
- **Linux/macOS**: `source .venv/bin/activate`
- **Windows**: `.venv\Scripts\activate`

When active, your prompt shows `(.venv)`; exit with `deactivate`. `make` picks up `pio` from PATH, so activate first, or skip activation with `make PIO=.venv/bin/pio build`. `.venv/` is gitignored. If pip install fails on the newest Python, create the venv with an older interpreter: `python3.13 -m venv .venv`.

**Good to know**

- Platforms, toolchains, and libraries always download into `~/.platformio` (PlatformIO core dir), shared by both options, so only the `pio` CLI itself is isolated by a venv. First build of each board downloads its toolchain (needs network).
- Use one `pio` at a time; do not activate the venv while another `pio` is first on PATH. Check with `which pio`.
- Recommendation: use Option A if PlatformIO is already installed; use Option B for a clean setup or to pin a version.

## Development

Built with PlatformIO and developed in VS Code.

```bash
make build      # build default env balance-xiao_s3
make upload     # upload default env
make monitor    # serial monitor
make test       # native unit tests
make help       # all targets
```

Pick another program/board with `ENV=<program>-<board>`, see below.

## Build and upload image by environment

Each program and board combination is a PlatformIO environment. Environments are named `<program>-<board>`; the default is `balance-xiao_s3`. Use the Makefile for a convenient interface to common build tasks. Run `make help` to see all available targets. Plain `make build` (or `pio run`) builds only the default environment; use `make build-all` to build every one.

| Environment | Program | Board | Build | Upload | Monitor |
| --- | --- | --- | --- | --- | --- |
| balance-xiao_s3 | balance | Seeed XIAO ESP32S3 | `make build-balance-xiao_s3` | `make upload-balance-xiao_s3` | `make monitor-balance-xiao_s3` (default) |
| balance-esp32dev | balance | Generic ESP32 dev board | `make build-balance-esp32dev` | `make upload-balance-esp32dev` | `make monitor-balance-esp32dev` |
| blink-uno | blink | Arduino Uno R3 (ATmega328P) | `make build-blink-uno` | `make upload-blink-uno` | `make monitor-blink-uno` |
| blink-uno_r4_wifi | blink | Arduino UNO R4 WiFi | `make build-blink-uno_r4_wifi` | `make upload-blink-uno_r4_wifi` | `make monitor-blink-uno_r4_wifi` |
| blink-attiny85 | blink | ATtiny85 | `make build-blink-attiny85` | `make upload-blink-attiny85` | n/a (no hardware UART) |
| blink-xiao_s3 | blink | Seeed XIAO ESP32S3 | `make build-blink-xiao_s3` | `make upload-blink-xiao_s3` | `make monitor-blink-xiao_s3` |
| blink-esp32dev | blink | Generic ESP32 dev board | `make build-blink-esp32dev` | `make upload-blink-esp32dev` | `make monitor-blink-esp32dev` |
| test | unit tests | native (host) | `make test` | n/a | n/a |

### Common commands

```bash
# Build a specific environment
make build ENV=blink-uno

# Upload to a device on a specific port
make upload ENV=blink-uno PORT=/dev/ttyUSB0

# Monitor serial output
make monitor ENV=blink-uno PORT=/dev/ttyUSB0

# Build all environments
make build-all

# Clean build artifacts
make clean-all

# List all environments
make envs
```

Without Make, use PlatformIO directly:
```bash
pio run -e <env>
pio run -e <env> -t upload [--upload-port <port>]
pio device monitor -e <env> [--port <port>]
```

### Notes

- Upload speed is 921600 baud for ESP32 boards; serial monitor runs at 115200 baud.
- The `balance` program is not built for Uno R3, Uno R4 WiFi, or ATtiny85 (insufficient RAM and no IMU support).
- Uno R3 and Uno R4 WiFi are different boards: R3 (`blink-uno`) is an ATmega328P uploaded with avrdude; R4 WiFi (`blink-uno_r4_wifi`) is a Renesas RA4M1 with native USB. Uploading an R4 with `blink-uno` fails with `stk500_getsync ... not in sync`; use the R4 env instead.
- ATtiny85 uploads through an ISP programmer. The default protocol in `platformio.ini` is `usbasp`; change `upload_protocol` in `[board_attiny85]` to match your programmer (e.g., `arduinoisp`, `stk500v1`). The board uses an 8 MHz internal clock.
- `make clean` cleans build artifacts for one environment; `make clean-all` cleans all environments.
- For instructions on adding a new program or board, see `CLAUDE.md`.

## Project Structure

- `programs/<name>/main.cpp` – one entry point per program (`balance`, `blink`)
- `lib/` – shared libraries usable by every program (servo, imu, controller planned; see CLAUDE.md)
- `include/` – shared headers (create when needed)
- `test/` – unit tests for the native `test` env (create when needed)
- `platformio.ini` – board sections and `<program>-<board>` envs
- `Makefile` – wrapper around `pio`
- `CLAUDE.md` – project guide for Claude Code

## Key Libraries

- PlatformIO Arduino framework for each board (atmelavr, renesas-ra, espressif32)
- Planned shared libs in `lib/`: STS3032 servo bus driver, IMU driver, PID controller (balance program)

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
