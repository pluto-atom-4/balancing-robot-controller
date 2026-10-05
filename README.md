# Balancing Robot Controller

Multi-program PlatformIO project for several dev boards (Arduino Uno R3, Arduino UNO R4 WiFi, Seeed XIAO ESP32S3, Seeed XIAO ESP32C3, generic ESP32, ATtiny85). Its main program is `balance`, self-balancing robot firmware targeting the Seeed XIAO ESP32-C3. Each program lives in `programs/<name>/` and shares code from `lib/`.

This is the **C++ / PlatformIO firmware** repo. The sibling Python repo [`pluto-atom-4/freecad-workspace`](https://github.com/pluto-atom-4/freecad-workspace) holds the Webots simulation, the Python PID/LQR reference controllers and the Python HAL. Python is the source of truth for the controller gains and the golden test vectors; this repo will keep committed, **generated** copies of them (never hand-edit those; regenerate from Python). See [Status and plan](#status-and-plan-stage-f).

## Hardware (balance program, planned)

- **MCU**: Seeed Studio XIAO ESP32-C3 only. The ESP32-S3 is a future enhancement built on the C3 base.
- **IMU**: MPU-6050 over I2C (the C3 has no built-in IMU). SDA=D4 (GPIO6), SCL=D5 (GPIO7).
- **Wheel servos**: Dynamixel XL330 (bench, now) and Feetech STS3032 (planned for the final robot), behind one motor-backend interface. The servo bus is `Serial1` on GPIO20 (RX) / GPIO21 (TX) through an FE-URT-1 adapter, as `tilt_servo` does today. Dynamixel2Arduino builds on the C3 in `tilt_servo`; an STS3032 driver on the C3 is untested.
- **Communication**: USB-C (programming, serial monitor over native USB CDC), serial bus for servo control

Nothing here has been validated on real hardware.

### Supported boards

- Seeed XIAO ESP32C3 (`xiao_c3`, `tilt_servo` + blink today; `balance` planned)
- Seeed XIAO ESP32S3 (`xiao_s3`, legacy balance stub + blink)
- Generic ESP32 dev board (`esp32dev`, legacy balance stub + blink)
- Arduino Uno R3 ATmega328P (`uno`, blink only)
- Arduino UNO R4 WiFi (`uno_r4_wifi`, blink only) — uses the `renesas-ra` platform, maintained separately from PlatformIO's core `atmelavr`/`espressif32` platforms; PlatformIO auto-installs it on first build of a `uno_r4_wifi` env (needs network), or pre-install with `pio pkg install --platform renesas-ra`.
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
- The `renesas-ra` platform (used by `uno_r4_wifi`, i.e. the Arduino UNO R4 WiFi board) is a separate community platform, not bundled with PlatformIO Core like `atmelavr` or `espressif32`. It installs automatically on first build of that env; to install ahead of time run `pio pkg install --platform renesas-ra`.

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

Each program and board combination is a PlatformIO environment. Environments are named `<program>-<board>`; the default is `balance-xiao_s3` (a stub, see below). Use the Makefile for a convenient interface to common build tasks. Run `make help` to see all available targets. Plain `make build` (or `pio run`) builds only the default environment; use `make build-all` to build every one.

| Environment | Program | Board | Build | Upload | Monitor |
| --- | --- | --- | --- | --- | --- |
| balance-xiao_s3 | balance (stub) | Seeed XIAO ESP32S3 | `make build-balance-xiao_s3` | `make upload-balance-xiao_s3` | `make monitor-balance-xiao_s3` (default) |
| balance-esp32dev | balance (stub) | Generic ESP32 dev board | `make build-balance-esp32dev` | `make upload-balance-esp32dev` | `make monitor-balance-esp32dev` |
| tilt_servo-xiao_c3 | tilt_servo | Seeed XIAO ESP32C3 | `make build-tilt_servo-xiao_c3` | `make upload-tilt_servo-xiao_c3` | `make monitor-tilt_servo-xiao_c3` |
| blink-xiao_c3 | blink | Seeed XIAO ESP32C3 | `make build-blink-xiao_c3` | `make upload-blink-xiao_c3` | `make monitor-blink-xiao_c3` |
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
- The `balance` program is not built for Uno R3, Uno R4 WiFi, or ATtiny85 (insufficient RAM and no IMU support). Today `programs/balance/main.cpp` is a stub that only prints an init line; the C3 balance firmware is planned (see below).
- Planned envs, not in `platformio.ini` yet ([#23](https://github.com/pluto-atom-4/balancing-robot-controller/issues/23)): `balance-xiao_c3` (LQR default) and `balance-xiao_c3-pid` (PID). #23 may rename the hyphenated one. Retiring the legacy `balance-xiao_s3` and `balance-esp32dev` envs is pending the maintainer's OK; they are **not** removed.
- `tilt_servo` is XIAO C3 only: it reads an MPU6050, shows a plate on an SSD1306 OLED, and drives a Dynamixel XL330 through an FE-URT-1. It has not been validated on hardware.
- Uno R3 and Uno R4 WiFi are different boards: R3 (`blink-uno`) is an ATmega328P uploaded with avrdude; R4 WiFi (`blink-uno_r4_wifi`) is a Renesas RA4M1 with native USB. Uploading an R4 with `blink-uno` fails with `stk500_getsync ... not in sync`; use the R4 env instead.
- ATtiny85 uploads through an ISP programmer. The default protocol in `platformio.ini` is `usbasp`; change `upload_protocol` in `[board_attiny85]` to match your programmer (e.g., `arduinoisp`, `stk500v1`). The board uses an 8 MHz internal clock.
- `make clean` cleans build artifacts for one environment; `make clean-all` cleans all environments.
- For instructions on adding a new program or board, see `CLAUDE.md`.

## Project Structure

- `programs/<name>/main.cpp` – one entry point per program (`balance` stub, `blink`, `tilt_servo`)
- `programs/<name>/wokwi.toml`, `diagram.json` – Wokwi simulation config (`blink`, `tilt_servo`)
- `lib/` – shared libraries usable by every program. Exists today: `lib/tilt` (header-only pitch math). Planned for balance: see below
- `include/` – shared headers (create when needed)
- `test/` – unit tests for the native `test` env (today: `test/test_tilt`)
- `platformio.ini` – board sections and `<program>-<board>` envs
- `Makefile` – wrapper around `pio`
- `CLAUDE.md` – project guide for Claude Code

## Key Libraries

- PlatformIO Arduino framework for each board (atmelavr, renesas-ra, espressif32)
- `tilt_servo-xiao_c3` pins its third-party libs in `platformio.ini`: Adafruit MPU6050 / Unified Sensor / BusIO / GFX / SSD1306 and Dynamixel2Arduino
- Planned in-repo libs for balance (not present yet): `hal_iface` (C++ mirror of the Python HAL contract), `balance_core` (header-only float32 PID + LQR, no heap), `loop_stats` (loop period tracker), `dxl_units` (rad/s <-> Dynamixel velocity units; constants unverified against the XL330 e-Manual)

## Simulation (Wokwi)

Run a program in the Wokwi simulator without hardware (needs `wokwi-cli` or the CLion Wokwi plugin):

```bash
make sim                    # tilt_servo-xiao_c3; no servo in the sim, so the firmware runs read-only
make sim-blink-xiao_c3      # serial-only control sketch
```

Config lives in `programs/<program>/` (`wokwi.toml`, `diagram.json`). The C3 firmware prints over USB CDC, so each C3 `diagram.json` must set `"serialInterface": "USB_SERIAL_JTAG"` on the board part. The CLI never exits on its own, so `make sim` ends with the timeout exit code 42. The simulator does not cover the servo, and it does not validate balance. A Wokwi config for the planned `balance-xiao_c3` env is tracked in [#33](https://github.com/pluto-atom-4/balancing-robot-controller/issues/33).

## Status and plan (Stage F)

**Nothing in this repo has been validated on real hardware.** `programs/balance` is a stub; the items below are **planned**, not implemented.

- **Source of truth and generated files.** Controller gains and golden parity vectors come from the Python repo. Planned generated files (do not hand-edit): `lib/balance_core/balance_gains_generated.h` (LQR gain K, resting-pitch offset `THETA_REF`, output limit, PID gains, schema / contract-version / gains-hash stamp) and `test/test_balance_parity/vectors_generated.inc` (golden vectors as plain C arrays, so native Unity tests need no Python or scipy). They will be produced by a Python exporter, `export_cpp.py` ([freecad-workspace#353](https://github.com/pluto-atom-4/freecad-workspace/issues/353)), run manually against a local checkout of this repo, with a `--check` drift detector. There is no cross-repo CI. Workflow: change gains in Python, run the exporter, commit the regenerated copies here.
- **HAL contract.** Hand-mirrored in both repos (`HAL_CONTRACT_VERSION` in Python, `kContractVersion` in the planned `lib/hal_iface/hal_iface.h`). A change to units (rad, rad/s), pitch sign, `THETA_REF` or dt semantics bumps the version in both. dt semantics (v1): `dt == 0` is P-only, `dt < 0` gives output 0 plus a fault flag, `wait_next_tick` returning `< 0` means stop.
- **Controllers and tests.** PID and LQR, switchable at compile time (`-D BALANCE_CONTROLLER_LQR` / `-D BALANCE_CONTROLLER_PID`). A native parity test (`test/test_balance_parity`, run with `pio test -e test`) is planned as the gate against the Python vectors.
- **Diagnostics (planned, issues not filed yet).** `c3_probe` (times IMU read, servo write, soft-float control step, free heap, loop period on the real C3) and `c3_facts` (gyro bias, resting pitch and sign, servo limits, a capped velocity-unit ramp with the servo off the robot), each with its own env, plus a native vector round-trip test.
- **Servo family.** XL330 now, STS3032 later, behind a motor-backend interface. The Dynamixel-specific issues #27 and #31 are pending a revision for that split; the STS3032 design is an open decision.
- **Known risks.** The ESP32-C3 reportedly has no hardware FPU (not yet verified), so float32 would be soft-float with unmeasured cost; loop timing and jitter on the real C3 are unmeasured; sim-derived constants (`K`, `THETA_REF = -0.1086 rad`) will not match the real robot; the PID sign convention is unverified even in simulation; Dynamixel runaway or torque left on after a fault is a safety risk, so a tilt cutoff and fail-safe are required.

Tracking issues:

- Stage F parent (Python repo): https://github.com/pluto-atom-4/freecad-workspace/issues/338
- Parent of parent (Webots simulation): https://github.com/pluto-atom-4/freecad-workspace/issues/10
- C++ umbrella (this repo): https://github.com/pluto-atom-4/balancing-robot-controller/issues/22, children #23–#35
- Python exporter (planned): https://github.com/pluto-atom-4/freecad-workspace/issues/353

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
