# Balancing Robot Controller

Multi-program PlatformIO project for several dev boards (Arduino Uno R3, Arduino UNO R4 WiFi, Seeed XIAO ESP32S3, Seeed XIAO ESP32C3, generic ESP32, ATtiny85). Its main program is `balance`, self-balancing robot firmware targeting the Seeed XIAO ESP32-C3. Each program lives in `programs/<name>/` and shares code from `lib/`.

This is the **C++ / PlatformIO firmware** repo. The sibling Python repo [`pluto-atom-4/freecad-workspace`](https://github.com/pluto-atom-4/freecad-workspace) holds the Webots simulation, the Python PID/LQR reference controllers and the Python HAL. Python is the source of truth for the controller gains and the golden test vectors; this repo keeps committed, **generated** copies of them (never hand-edit those; regenerate from Python). See [Status and plan](#status-and-plan-stage-f).

## Hardware (balance program)

- **MCU**: Seeed Studio XIAO ESP32-C3 only. The ESP32-S3 is a future enhancement built on the C3 base.
- **IMU**: MPU-6050 over I2C (the C3 has no built-in IMU). SDA=D4 (GPIO6), SCL=D5 (GPIO7).
- **Wheel servos**: Dynamixel XL330 (bench, now) and Feetech STS3032 (planned for the final robot), behind one servo-family interface (`IWheelServo` in `lib/wheel_servo/wheel_servo.h`). The XL330 driver is `programs/balance/dxl_wheels.h` (Dynamixel2Arduino, velocity mode, torque-off first). The servo bus is `Serial1` on GPIO20 (RX) / GPIO21 (TX) through an FE-URT-1 adapter, as `tilt_servo` does. An STS3032 driver does not exist yet and is untested on the C3.
- **Communication**: USB-C (programming, serial monitor over native USB CDC), serial bus for servo control

Nothing here has been validated on real hardware.

### Supported boards

- Seeed XIAO ESP32C3 (`xiao_c3`, `balance` (LQR and PID envs), `tilt_servo`, blink)
- Seeed XIAO ESP32S3 (`xiao_s3`, blink only; a future enhancement for `balance`, built on the C3 base)
- Generic ESP32 dev board (`esp32dev`, blink only)
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
make build      # build default env balance-xiao_c3
make upload     # upload default env
make monitor    # serial monitor
make test       # native unit tests (pio test -e test)
make help       # all targets
```

Pick another program/board with `ENV=<program>-<board>`, see below.

## Build and upload image by environment

Each program and board combination is a PlatformIO environment. Environments are named `<program>-<board>`; the default is `balance-xiao_c3` (LQR controller). Use the Makefile for a convenient interface to common build tasks. Run `make help` to see all available targets. Plain `make build` (or `pio run`) builds only the default environment; use `make build-all` to build every one.

| Environment | Program | Board | Build | Upload | Monitor |
| --- | --- | --- | --- | --- | --- |
| balance-xiao_c3 | balance (LQR, `-D BALANCE_CONTROLLER_LQR`) | Seeed XIAO ESP32C3 | `make build-balance-xiao_c3` | `make upload-balance-xiao_c3` | `make monitor-balance-xiao_c3` (default) |
| balance-xiao_c3-pid | balance (PID, `-D BALANCE_CONTROLLER_PID`) | Seeed XIAO ESP32C3 | `make build-balance-xiao_c3-pid` | `make upload-balance-xiao_c3-pid` | `make monitor-balance-xiao_c3-pid` |
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
- The `balance` program is built only for the XIAO C3 envs (`balance-xiao_c3`, `balance-xiao_c3-pid`), not for Uno R3, Uno R4 WiFi, ATtiny85, XIAO S3 or generic ESP32 (insufficient RAM or no IMU support, or out of scope). `programs/balance/main.cpp` is a control loop (IMU, controller, wheel pair, latched supervisor); it has not been validated on real hardware (see Status).
- Exactly one of `-D BALANCE_CONTROLLER_LQR` (env `balance-xiao_c3`) or `-D BALANCE_CONTROLLER_PID` (env `balance-xiao_c3-pid`) must be set; `programs/balance/main.cpp` stops the build with `#error` if neither or both are. The legacy `balance-xiao_s3` and `balance-esp32dev` envs were retired in [#46](https://github.com/pluto-atom-4/balancing-robot-controller/issues/46). The default env is `balance-xiao_c3`.
- C++ standard: `platformio.ini` passes `-std=gnu++17` for ESP32 boards and the native `test` env, but the C3 firmware compile also carries `-std=gnu++11` from the platform, so headers in `lib/` that firmware includes must stay valid C++11 (see the note in `lib/balance_core/balance_stamp.h`, [#56](https://github.com/pluto-atom-4/balancing-robot-controller/issues/56)). The native tests do not catch C++11 violations; a firmware build (`pio run -e balance-xiao_c3`) does.
- `tilt_servo` is XIAO C3 only: it reads an MPU6050, shows a plate on an SSD1306 OLED, and drives a Dynamixel XL330 through an FE-URT-1. It has not been validated on hardware.
- Uno R3 and Uno R4 WiFi are different boards: R3 (`blink-uno`) is an ATmega328P uploaded with avrdude; R4 WiFi (`blink-uno_r4_wifi`) is a Renesas RA4M1 with native USB. Uploading an R4 with `blink-uno` fails with `stk500_getsync ... not in sync`; use the R4 env instead.
- ATtiny85 uploads through an ISP programmer. The default protocol in `platformio.ini` is `usbasp`; change `upload_protocol` in `[board_attiny85]` to match your programmer (e.g., `arduinoisp`, `stk500v1`). The board uses an 8 MHz internal clock.
- `make clean` cleans build artifacts for one environment; `make clean-all` cleans all environments.
- For instructions on adding a new program or board, see `CLAUDE.md`.

## Project Structure

- `programs/<name>/main.cpp` – one entry point per program (`balance`, `blink`, `tilt_servo`). `programs/balance/` also holds `config.h`, `imu_mpu6050.h`, `dxl_wheels.h` (Arduino-bound code only)
- `programs/<name>/wokwi.toml`, `diagram.json` – Wokwi simulation config (`blink`, `tilt_servo`, `balance`)
- `lib/` – shared libraries usable by every program: `hal_iface`, `balance_core` (incl. generated `balance_gains_generated.h` and `balance_stamp.h`), `loop_stats`, `dxl_units`, `imu_math`, `wheel_servo`, `balance_supervisor`, `tilt`. Details under Key Libraries
- `include/` – shared headers (create when needed)
- `test/` – unit tests for the native `test` env (11 suites, 131 `RUN_TEST` cases: balance_core, balance_parity, balance_stamp, balance_supervisor, dxl_units, hal_iface, imu_math, loop_stats, tilt, vector_roundtrip, wheel_pair)
- `platformio.ini` – board sections and `<program>-<board>` envs
- `Makefile` – wrapper around `pio`
- `CLAUDE.md` – project guide for Claude Code

## Key Libraries

- PlatformIO Arduino framework for each board (atmelavr, renesas-ra, espressif32)
- `tilt_servo-xiao_c3` pins its third-party libs in `platformio.ini`: Adafruit MPU6050 / Unified Sensor / BusIO / GFX / SSD1306 and Dynamixel2Arduino
- In-repo libs for balance (all hardware-neutral and natively tested; none validated on hardware): `hal_iface` (C++ mirror of the Python HAL contract, `kContractVersion`), `balance_core` (header-only float32 PID + LQR, no heap), `balance_stamp.h` (schema / contract / gains-hash stamp check), `loop_stats` (loop period tracker), `dxl_units` (rad/s <-> Dynamixel velocity units; constants UNVERIFIED against the XL330 e-Manual), `imu_math` (pure MPU-6050 math), `wheel_servo` (`IWheelServo` interface and `WheelPair` safety state machine), `balance_supervisor` (pure safety state machine), `tilt` (pitch math for `tilt_servo`)
- `balance-xiao_c3*` pin Adafruit MPU6050 / Unified Sensor / BusIO and Dynamixel2Arduino (no OLED libs), via `[balance_c3_common]` in `platformio.ini`

## Simulation (Wokwi)

Run a program in the Wokwi simulator without hardware (needs `wokwi-cli` or the CLion Wokwi plugin):

```bash
make sim                    # tilt_servo-xiao_c3; no servo in the sim, so the firmware runs read-only
make sim-blink-xiao_c3      # serial-only control sketch
make sim-balance-xiao_c3    # balance firmware (LQR): boot and wiring check only
```

Config lives in `programs/<program>/` (`wokwi.toml`, `diagram.json`). The C3 firmware prints over USB CDC, so each C3 `diagram.json` must set `"serialInterface": "USB_SERIAL_JTAG"` on the board part. The CLI never exits on its own, so `make sim` ends with the timeout exit code 42. The simulator does not cover the servo, and it does not validate balance. The `balance` sim ([#33](https://github.com/pluto-atom-4/balancing-robot-controller/issues/33)) is an ESP32-C3 devkit plus an MPU-6050 and no servo, so it checks only boot and I2C wiring; it is configured for the LQR env only, and `make sim-balance-xiao_c3-pid` is refused by design because the PID firmware path is not in its `wokwi.toml`. With no servo present the wheel startup fails and latches; this was observed in the simulator (`startup FAILED`, `LATCHED cause=startup`, then `gave up: REMOVE SERVO POWER`), and the IMU is still read and printed with state Latched.

## Status and plan (Stage F)

**Nothing in this repo has been validated on real hardware.** `programs/balance` is a control loop built from unit-tested libs; native tests and Wokwi do not validate balance on hardware. Implemented: libs, envs, generated files, parity test, Wokwi boot check. Still planned: `c3_probe` ([#43](https://github.com/pluto-atom-4/balancing-robot-controller/issues/43)) and `c3_facts` ([#44](https://github.com/pluto-atom-4/balancing-robot-controller/issues/44)) have no code yet; the hardware checklist ([#34](https://github.com/pluto-atom-4/balancing-robot-controller/issues/34)) is for a human; the STS3032 driver; a BOOT-button hardware kill.

- **Source of truth and generated files.** Controller gains and golden parity vectors come from the Python repo. Committed, **generated** copies (do not hand-edit): `lib/balance_core/balance_gains_generated.h` (LQR gain K, resting-pitch offset `THETA_REF`, output limit, PID gains, schema / contract-version / gains-hash stamp) and `test/test_balance_parity/vectors_generated.inc` (golden vectors as plain C arrays, so native Unity tests need no Python or scipy). `.gitattributes` pins both to LF because the exporter `--check` wants byte-identical output. They are produced by `export_cpp.py` in freecad-workspace ([#353](https://github.com/pluto-atom-4/freecad-workspace/issues/353), landed in freecad-workspace PR #368), run manually against a local checkout of this repo, with a `--check` drift detector: `mamba run -n pendulum-tools python3 07_Simulation/hal/export_cpp.py --cpp-repo <this repo> [--check]` (run from the `inverted-pendulum-project` directory of the Python repo; the `--check` form was run against this repo and reported both generated files byte-identical). The values are SIM-derived. There is no cross-repo CI. Workflow: change gains in Python, run the exporter, commit the regenerated copies here.
- **HAL contract.** Hand-mirrored in both repos (`HAL_CONTRACT_VERSION` in Python, `kContractVersion` in `lib/hal_iface/hal_iface.h`). A change to units (rad, rad/s), pitch sign, `THETA_REF` or dt semantics bumps the version in both. dt semantics (v1): `dt == 0` is P-only, `dt < 0` gives output 0 plus a fault flag, `wait_next_tick` returning `< 0` means stop.
- **Controllers and tests.** PID and LQR, switchable at compile time (`-D BALANCE_CONTROLLER_LQR` / `-D BALANCE_CONTROLLER_PID`). The native parity test `test/test_balance_parity` (run with `pio test -e test`, no hardware, no Python) compares the C++ core against the Python vectors; it is the parity gate. `test/test_vector_roundtrip` ([#42](https://github.com/pluto-atom-4/balancing-robot-controller/issues/42)) checks the stamps and the generated vectors against the core.
- **Diagnostics (planned, no code yet).** `c3_probe` ([#43](https://github.com/pluto-atom-4/balancing-robot-controller/issues/43): times IMU read, servo write, soft-float control step, free heap, loop period on the real C3) and `c3_facts` ([#44](https://github.com/pluto-atom-4/balancing-robot-controller/issues/44): gyro bias, resting pitch and sign, servo limits, a capped velocity-unit ramp with the servo off the robot), each with its own env once written.
- **Servo family.** XL330 now (`programs/balance/dxl_wheels.h`, velocity mode), STS3032 later; both hide behind `IWheelServo` (`lib/wheel_servo/wheel_servo.h`). The STS3032 driver does not exist; its design is an open decision. The servo baud is 57600 (`kServoBaud` in `programs/balance/config.h`); a requested 2 Mbaud is untested.
- **Safety design (UNVERIFIED on hardware).** Wheel torque is switched off first thing in `setup()`, before IMU bring-up and the blocking gyro calibration. Motors stay off until the pitch stays within the lean gate for a hold time. A latched supervisor (`lib/balance_supervisor`) turns torque off and stays latched until board reset (no re-arm) on: tilt cutoff, repeated IMU read failures, non-finite or faulted controller output, repeated wheel write failures, a tick gap above the stall threshold, a failed startup, or the serial command `x`/`X`. All limits are in `programs/balance/config.h`, are SIM-derived or guessed, and are marked UNVERIFIED / TODO(human). A true hang of the MCU cannot be handled in software; the stall check only fires after the loop resumes. A BOOT-button hardware kill is a TODO, not implemented. Serial lines: `[bal]`, `[loop]`, `[imu]`, `[wheel]`, `[safe]`.
- **Known risks.** The ESP32-C3 reportedly has no hardware FPU (still not verified), so float32 would be soft-float with unmeasured cost; loop timing and jitter on the real C3 are unmeasured (the 50 Hz period and jitter budget come from simulation analysis, freecad-workspace#345); sim-derived constants (`K`, `THETA_REF = -0.1086 rad`) will not match the real robot; LQR and PID give opposite signs for the same tilt and the sign convention is unverified (freecad-workspace#359, checked by [#34](https://github.com/pluto-atom-4/balancing-robot-controller/issues/34)); servo IDs, wheel signs and the XL330 unit constants are unconfirmed; Dynamixel runaway or torque left on after a fault is a safety risk (mitigated in software as described above, not proven).

Tracking issues:

- Stage F parent (Python repo): https://github.com/pluto-atom-4/freecad-workspace/issues/338
- Parent of parent (Webots simulation): https://github.com/pluto-atom-4/freecad-workspace/issues/10
- C++ umbrella (this repo): https://github.com/pluto-atom-4/balancing-robot-controller/issues/22, children #23–#35, #42–#44, #46, #56
- Python exporter: https://github.com/pluto-atom-4/freecad-workspace/issues/353

## Verification status of this document

The commands in this README were checked statically (file paths, env names and make targets compared with `platformio.ini` and `Makefile`). Commands actually run for the last docs update: `pio test -e test` (131 cases, all passed); `pio run -e balance-xiao_c3`, `pio run -e balance-xiao_c3-pid`, `pio run -e tilt_servo-xiao_c3` and `pio run -e blink-xiao_c3` (all succeeded); and a verbose build of `balance-xiao_c3` to confirm that both `-std=gnu++11` and `-std=gnu++17` appear in the C3 compile line. Not re-run for this update: `make sim-balance-xiao_c3` (last run for #33, PR #55) and the exporter `--check` (last run for #29, PR #51). Any command not listed was not run. Nothing was run on real hardware.

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
