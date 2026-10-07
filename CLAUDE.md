# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Multi-program, multi-board PlatformIO project. Main program: self-balancing robot firmware (`balance`). This repo is the **C++ / PlatformIO firmware** side; the sibling Python repo [`pluto-atom-4/freecad-workspace`](https://github.com/pluto-atom-4/freecad-workspace) (project dir `inverted-pendulum-project`) holds the Webots simulation, the Python PID/LQR reference controllers and the Python HAL. Python there, C++ here.

**Status: `programs/balance/main.cpp` is a balance control loop** (IMU -> PID/LQR -> wheel pair, with a latched safety supervisor), built from natively tested libs under umbrella issue [#22](https://github.com/pluto-atom-4/balancing-robot-controller/issues/22). **Nothing in it is validated on real hardware.** `balance`, `tilt_servo` and `blink` build in `platformio.ini`; `tilt_servo` and `blink` are not hardware-validated either. Still planned or open: `c3_probe` (#43) builds but has never been run on hardware, `c3_facts` (#44) has no code, the hardware checklist (#34) is for a human, a BOOT-button kill is a TODO, the STS3032 driver is untested, and the ESP32-S3 is a future enhancement.

### Target hardware (balance)

- **Board: Seeed XIAO ESP32-C3 only.** The ESP32-S3 is a *future enhancement* built on the C3 base. Do not add multi-board support or S3 branches for `balance`.
- **IMU**: MPU-6050 over I2C (the C3 has no built-in IMU). I2C SDA=D4 (GPIO6), SCL=D5 (GPIO7), as in `programs/tilt_servo/main.cpp`.
- **Wheel servos**: Dynamixel **XL330 on the bench now, Feetech STS3032 planned for the final robot.** The servo family hides behind `wheel_servo::IWheelServo` (`lib/wheel_servo/wheel_servo.h`) so one can replace the other. The XL330 driver is `programs/balance/dxl_wheels.h` (Dynamixel2Arduino, velocity mode; never writes Velocity Limit; writes Operating Mode only if it is not already velocity). The servo bus uses `Serial1` on GPIO20 (RX, D7) / GPIO21 (TX, D6) through an FE-URT-1 adapter, as `tilt_servo` does (`tilt_servo-xiao_c3` uses Dynamixel2Arduino on the C3). An STS3032 driver does not exist and is **untested** on the C3; its final design is an open decision.
- Both PID and LQR are supported, switchable at compile time (`-D BALANCE_CONTROLLER_LQR` / `-D BALANCE_CONTROLLER_PID`).

The older text "balance on XIAO S3 Sense with built-in IMU and two STS3032 servos" is stale. The legacy `balance-xiao_s3` and `balance-esp32dev` stub envs were retired in [#46](https://github.com/pluto-atom-4/balancing-robot-controller/issues/46).

Supported boards: Arduino Uno R3 ATmega328P (`uno`, atmelavr), Arduino UNO R4 WiFi (`uno_r4_wifi`, renesas-ra), Seeed XIAO ESP32S3 (`xiao_s3`), generic ESP32 dev (`esp32dev`), ATtiny85 (`attiny85`, atmelavr), Seeed XIAO ESP32C3 (`xiao_c3`, espressif32). More boards can be added.

Programs: `balance` (control loop; built only for the XIAO C3 envs, not for Uno R3, Uno R4 WiFi, ATtiny85, XIAO S3 or generic ESP32), `blink` (portable example, all boards), `tilt_servo` (XIAO C3 only: MPU6050 tilt drives Dynamixel XL330 via FE-URT-1), `c3_probe` (diagnostic probe, XIAO C3 only).

Envs, named `<program>-<board>`: `balance-xiao_c3` (default, LQR, `-D BALANCE_CONTROLLER_LQR`), `balance-xiao_c3-pid` (PID, `-D BALANCE_CONTROLLER_PID`), `blink-uno`, `blink-uno_r4_wifi`, `blink-xiao_s3`, `blink-esp32dev`, `blink-xiao_c3`, `blink-attiny85`, `tilt_servo-xiao_c3`, `c3_probe-xiao_c3`. Native test env: `test` (`-std=gnu++17 -Wall -Wextra -ffp-contract=off`).

`programs/balance/main.cpp` fails with `#error` if neither or both of the two controller flags are set. `make sim-balance-xiao_c3` works; `make sim-balance-xiao_c3-pid` is refused by design (see Wokwi).

## Development Commands

Use `-e <program>-<board>` to pick an env. Default env: `balance-xiao_c3`.

**Build default env** (balance-xiao_c3)
```bash
pio run
```

**Build one env**
```bash
pio run -e balance-xiao_c3
pio run -e balance-xiao_c3-pid
pio run -e blink-uno
pio run -e blink-attiny85
```

**Upload one env**
```bash
pio run -e blink-uno -t upload
pio run -e blink-attiny85 -t upload
pio run -e balance-xiao_c3 -t upload
```

**Monitor serial output** (115200 baud)
```bash
pio device monitor -e blink-uno
```

**Run unit tests**
```bash
pio test -e test
```

**Simulate in Wokwi** (no hardware; needs `wokwi-cli` or the CLion Wokwi plugin)
```bash
make sim                       # tilt_servo-xiao_c3 (servo absent, firmware runs read-only)
make sim-blink-xiao_c3         # blink control sketch, serial-only check of the sim setup
make sim-balance-xiao_c3       # balance (LQR only): boot and I2C wiring check, no servo, not a balance validation
make sim-<program>-<board>     # any env whose programs/<program>/ has wokwi.toml + diagram.json
```
Each program keeps its own config in `programs/<program>/` (`wokwi.toml`, `diagram.json`), paths relative to the toml. `make sim-<env>` builds the env, then runs `wokwi-cli programs/<program>`. In CLion set Settings > Wokwi Simulator > config path to that program's `wokwi.toml`. Pass extra CLI flags with `WOKWI_ARGS="--timeout 10000"`. The C3 firmware uses USB CDC serial, so each C3 `diagram.json` must set `"serialInterface": "USB_SERIAL_JTAG"` on the board part or no serial output shows. The CLI never exits on its own (firmware loops), so `sim` ends with the timeout exit code 42. `sim-<env>` refuses an env unless `build/<env>/` appears in that program's `wokwi.toml`, so `sim-balance-xiao_c3-pid` is refused (the balance `wokwi.toml` points only at `build/balance-xiao_c3/`). The balance diagram is an ESP32-C3 devkit plus a `wokwi-mpu6050`, no servo: expect `[bal] boot LQR ...`; the wheel startup fails and latches with no servo, which was observed in the simulator (`startup FAILED`, `LATCHED cause=startup`, `gave up: REMOVE SERVO POWER`), and the IMU is still read and printed with state Latched.

**Clean build**
```bash
pio run -t clean
```

## Architecture

### Balance layout

Replaced the older `lib/servo`, `lib/imu`, `lib/controller` sketch. Headers use the `.h` extension, matching `lib/tilt/tilt.h`. All of it is unvalidated on hardware.

- `lib/hal_iface/hal_iface.h` – C++ mirror of the Python HAL contract (`kContractVersion`).
- `lib/balance_core/` – `balance_core.h` (header-only float32 PID + LQR core, no heap; gains are passed to the constructors, so the core does not depend on the generated header), `balance_gains_generated.h` (GENERATED, see below) and `balance_stamp.h` (pure schema / contract / gains-hash stamp check).
- `lib/loop_stats/` – loop period min/mean/max tracker (pure, natively tested).
- `lib/dxl_units/` – rad/s <-> Dynamixel raw velocity conversion (pure, natively tested). Its constants are UNVERIFIED against the XL330 e-Manual; the tracking issue requires a citation.
- `lib/imu_math/` – pure MPU-6050 math (degrees -> rad, pitch sign, gyro bias accumulator); natively tested.
- `lib/wheel_servo/` – servo-family-neutral `IWheelServo` interface plus `WheelPair`, which latches (torque off on both IDs, later writes refused) after repeated write failures or any begin failure; natively tested.
- `lib/balance_supervisor/` – pure latched safety state machine (Startup, Calibrating, WaitingForLean, Running, Latched); natively tested.
- `programs/balance/` – Arduino-bound code only: `imu_mpu6050.h`, `dxl_wheels.h` (XL330 `IWheelServo`), `config.h` (the one place for servo baud, IDs, wheel signs, loop and safety constants), `main.cpp` (control loop), `wokwi.toml` + `diagram.json`.
- C++ standard rule: the board sections pass `-std=gnu++17`, but the C3 firmware compile also carries `-std=gnu++11` from the platform (seen in the verbose compile line; see `lib/balance_core/balance_stamp.h`), so `lib/` headers that firmware includes must be valid C++11 (single-return `constexpr`, no `switch` in `constexpr`). The native `test` env is gnu++17 and does not catch this; only a firmware build does ([#56](https://github.com/pluto-atom-4/balancing-robot-controller/issues/56)).

### Project split and generated files

- **Python is the source of truth** for controller gains and golden parity vectors. This repo keeps committed, **generated** copies. **Do not hand-edit generated files**; regenerate them from the Python repo and commit the result.
- Generated files (committed; read-only; `.gitattributes` pins both to LF because the exporter `--check` wants byte-identical output):
  - `lib/balance_core/balance_gains_generated.h` – LQR gain K, resting-pitch offset `THETA_REF`, output limit, PID gains, plus a schema / contract-version / gains-hash stamp.
  - `test/test_balance_parity/vectors_generated.inc` – golden test vectors as plain C arrays, so native Unity tests need no Python or scipy.
- The Python exporter is `export_cpp.py` ([freecad-workspace#353](https://github.com/pluto-atom-4/freecad-workspace/issues/353), landed in freecad-workspace PR #368), run manually with a `--check` drift detector: `mamba run -n pendulum-tools python3 07_Simulation/hal/export_cpp.py --cpp-repo <this repo> [--check]` (run from the `inverted-pendulum-project` directory of the Python repo; the `--check` form was run against this repo and reported both generated files byte-identical). The generated values are SIM-derived. There is **no cross-repo CI**.
- The HAL contract is **hand-mirrored**: Python has `HAL_CONTRACT_VERSION`, this repo's `lib/hal_iface/hal_iface.h` carries `kContractVersion`. Any change to units (rad, rad/s), pitch sign, `THETA_REF`, or dt semantics must bump the version in both repos.
- dt semantics (contract v1): `dt == 0` means P-only; `dt < 0` means output 0 plus a fault flag; `wait_next_tick` returning `< 0` means stop.

### Balance safety design (UNVERIFIED on hardware)

- `setup()` switches wheel torque off first, before IMU bring-up and the blocking gyro calibration (`WheelPair::begin`, then `torqueOff` if it fails).
- Motors stay off until `|pitch - center| <= kStartupLeanGateRad` for `kLeanGateHoldTicks` ticks; `center` is the controller's own setpoint (`THETA_REF` for LQR, 0 for PID).
- `lib/balance_supervisor` latches (terminal, first cause wins, no re-arm; reset the board to re-arm) on: tilt cutoff (`kTiltCutoffRad`), consecutive IMU failures, non-finite or faulted controller output, `WheelPair` failure, stall (tick gap above `kStallUs`), startup failure, and the serial command `x`/`X` (`SerialKill`). Every path ends in `WheelPair::torqueOff()`, retried up to 5 times; if torque-off is not confirmed the firmware prints `REMOVE SERVO POWER`.
- A true hang cannot be handled in software; the stall check fires only after the loop resumes. A BOOT-button (GPIO9) hardware kill is a TODO in `config.h`, not implemented.
- Every constant in `programs/balance/config.h` is a SIM-derived or guessed value marked UNVERIFIED / TODO(human) (servo ID 2 and wheel signs are unconfirmed; `kJitterBudgetUs` comes from freecad-workspace#345, not a C3 measurement). `kWheelWritesEnabled = false` is a compute-and-log dry run.
- Do not weaken a kill path or the torque-off-first order without a design issue.

### Diagnostic / probe programs

Permanent diagnostic tools, each with its own env (`c3_probe` has code and builds, never run on hardware; `c3_facts` has no code yet):

- `c3_probe` ([#43](https://github.com/pluto-atom-4/balancing-robot-controller/issues/43)) – times the pieces of a balance loop on the real C3 (IMU read, servo write, soft-float PID/LQR step cost, free heap, loop period). Code exists in `programs/c3_probe/main.cpp` (env `c3_probe-xiao_c3`, builds; never run on hardware, so no measurement exists). Servo torque stays OFF by default (confirmed by read-back) and only goal velocity 0 is ever written; `PLATFORMIO_BUILD_FLAGS="-D C3PROBE_TORQUE_ON"` is a human opt-in (wheels off the ground). Output: one `C3PROBE <metric> <value> <unit>` line per metric over USB CDC. Run: `pio run -e c3_probe-xiao_c3 -t upload`, `pio device monitor -e c3_probe-xiao_c3 | tee c3probe.log`, then post a RESULTS comment on #43. It prints `__riscv_flen` but does not settle the FPU claim.
- `c3_facts` ([#44](https://github.com/pluto-atom-4/balancing-robot-controller/issues/44)) – static IMU facts (gyro bias in rad/s, resting pitch and its sign) and servo model, limits and a velocity-unit ramp. The ramp is capped around 20 % of the velocity limit, about 15 s, then torque off, with the servo off the robot and held down.
- The native vector round-trip test ([#42](https://github.com/pluto-atom-4/balancing-robot-controller/issues/42)) exists: `test/test_vector_roundtrip` (`pio test -e test`, no hardware).

`c3_facts` does not exist yet, so there is no run procedure for it. The `c3_probe` run procedure is in its bullet above and has not been executed on hardware.

### File Structure (what exists today)

- `programs/<name>/main.cpp` – One entry point per program (`src_dir = programs`)
  - `programs/balance/` – `main.cpp` (control loop), `config.h`, `imu_mpu6050.h`, `dxl_wheels.h`, `wokwi.toml` + `diagram.json`
  - `programs/blink/main.cpp` – Portable blink example
  - `programs/tilt_servo/main.cpp` – MPU6050 tilt drives Dynamixel XL330 (XIAO C3)
  - `programs/c3_probe/main.cpp` – Diagnostic probe (#43), builds, never run on hardware
  - `programs/<name>/wokwi.toml` + `diagram.json` – Wokwi sim config (`blink`, `tilt_servo`, `balance`)
- `include/` – Public headers
- `lib/` – Shared libraries, usable by every program in `programs/` (not tied to one). Each lib is its own folder `lib/<name>/` with `<name>.h` (+ `<name>.cpp` if needed; optional `library.json`). Exist: `hal_iface`, `balance_core`, `loop_stats`, `dxl_units`, `imu_math`, `wheel_servo`, `balance_supervisor` (see Balance layout) and `lib/tilt/tilt.h` (header-only pitch math, used by `tilt_servo`).
- `test/` – Unit tests, one folder per suite under `test/` (11 suites, 131 `RUN_TEST` cases): `test_balance_core`, `test_balance_parity` (+ generated `vectors_generated.inc`), `test_balance_stamp`, `test_balance_supervisor`, `test_dxl_units`, `test_hal_iface`, `test_imu_math`, `test_loop_stats`, `test_tilt`, `test_vector_roundtrip`, `test_wheel_pair`
- `platformio.ini` – Abstract board sections `[board_uno]`, `[board_uno_r4_wifi]`, `[board_xiao_s3]`, `[board_xiao_c3]`, `[board_esp32dev]`, `[board_attiny85]` (board, platform, flags); envs `[env:<program>-<board>]` combine a board section with a program via `build_src_filter = -<*> +<program/>`; `[balance_c3_common]` holds the `lib_deps` shared by the two balance envs.

## Key Constants/Configs

`tilt_servo` (XIAO C3, exists today; values from `programs/tilt_servo/main.cpp`):

- **Serial (USB CDC)**: 115200, monitoring only. `ARDUINO_USB_CDC_ON_BOOT=1` keeps `Serial` off GPIO20/21.
- **Servo bus**: `Serial1`, GPIO20 (RX) / GPIO21 (TX), 57600 baud, Dynamixel protocol 2.0, ID 1, through FE-URT-1 (direction handled in hardware, `DXL_DIR_PIN = -1`).
- **I2C**: SDA=D4 (GPIO6), SCL=D5 (GPIO7), 400 kHz.
- **Loop**: 50 Hz.

`balance` (XIAO C3; all values in `programs/balance/config.h`, all UNVERIFIED on hardware): USB CDC 115200; servo bus `Serial1` GPIO20/21 at 57600 baud (a 2 Mbaud request is untested), servo IDs 1 and 2 (ID 2 unconfirmed), wheel signs +1 / -1 (unconfirmed), I2C 400 kHz, loop 50 Hz nominal (`kLoopUs` = 20000 us; the real C3 rate, jitter and soft-float cost are to be measured by `c3_probe` (#43; built, not yet run)), wheel speed cap 1.0 rad/s, tilt cutoff 0.5 rad, startup lean gate 0.1 rad held 25 ticks, stall threshold 100 ms. Do not assume the ~100 Hz mentioned in older notes.

The STS3032 is the planned servo for the final robot; its C3 driver does not exist and is untested. (The old S3-plan figures, position 0-4095, speed 1-2047, are unverified and not used.)

## Build Notes

- `src_dir = programs`; each env selects its program via `build_src_filter`
- `xiao_s3`: board `seeed_xiao_esp32s3` (espressif32), upload speed 921600 baud; blink only today, a future enhancement for `balance`
- `xiao_c3`: board `seeed_xiao_esp32c3` (espressif32, RISC-V), upload speed 921600 baud, `ARDUINO_USB_MODE=1` + `ARDUINO_USB_CDC_ON_BOOT=1` (Serial over native USB). The target board for `balance`. The C3 reportedly has no hardware FPU, so float32 would be soft-float; this is **still not verified** and its cost is unmeasured (to be measured by `c3_probe`, #43 (built, not yet run)).
- `esp32dev`: generic ESP32 dev board (espressif32)
- ESP32 boards: `platformio.ini` passes `-std=gnu++17`, but the C3 firmware compile also carries `-std=gnu++11` from the platform, so firmware-included `lib/` headers must be valid C++11 (see the C++ standard rule under Balance layout). The native `test` env is `gnu++17`.
- `uno`: Arduino Uno R3 ATmega328P (atmelavr), no C++17 flag; 16MHz AVR, 2KB RAM, keep sketches small
- `uno_r4_wifi`: Arduino UNO R4 WiFi (renesas-ra, Renesas RA4M1, 32-bit ARM, native USB serial); different chip and uploader than Uno R3 — `blink-uno` on an R4 fails with `stk500_getsync ... not in sync`
- `attiny85`: ATtiny85 (atmelavr), 8MHz internal, 8KB flash, 512B RAM, no hardware UART so no Serial/monitor (blink guards Serial with `__AVR_ATtiny85__`), uploads via ISP programmer (`upload_protocol = usbasp` default in platformio.ini, change to match hardware)
- `balance` is built only for the XIAO C3 envs (not for `uno`, `uno_r4_wifi`, `attiny85`, `xiao_s3`, `esp32dev`)
- External dependencies are pinned per env in `platformio.ini` (`tilt_servo-xiao_c3`: Adafruit MPU6050 / Unified Sensor / BusIO / GFX / SSD1306 and Dynamixel2Arduino; `balance-xiao_c3*` via `[balance_c3_common]`: MPU6050 / Unified Sensor / BusIO and Dynamixel2Arduino)

## Common Edits

- **Tune PID/LQR gains**: do **not** edit `balance_gains_generated.h` by hand. Change the gains in the Python repo and regenerate (see "Project split and generated files").
- **Change balance setpoint**: the resting-pitch offset `THETA_REF` is also generated from Python. The sim value (`-0.1086 rad`) will not match the real robot; the real equilibrium angle must be measured.
- **Tilt/IMU math today**: `lib/tilt/tilt.h` (complementary filter, pitch, tick conversion) and `programs/tilt_servo/main.cpp`.
- **Servo command mapping**: balance XL330 calls are in `programs/balance/dxl_wheels.h` (`IWheelServo`); wheel IDs, signs, caps and safety limits are in `programs/balance/config.h`. `tilt_servo` has its own Dynamixel2Arduino calls in `programs/tilt_servo/main.cpp`. An STS3032 backend would implement `IWheelServo`; it does not exist yet.
- **Shared lib change**: Edits in `lib/` affect all programs; rebuild each env
- **Balance safety limits** (`kTiltCutoffRad`, lean gate, `kStallUs`, fail counts): in `programs/balance/config.h`; change only with a design issue (see Balance safety design).

## Shared libraries (lib/)

- Any program uses a lib by `#include <name.h>` (or `"name.h"`); PlatformIO finds it automatically (LDF), and only libs actually included are linked, so unused libs cost nothing.
- Keep libs hardware-neutral (no board-specific pins/APIs), or guard with `#if defined(ARDUINO_ARCH_AVR)` / `ARDUINO_ARCH_ESP32` / `ARDUINO_ARCH_RENESAS_UNO` / `__AVR_ATtiny85__`.
- Board-limited lib (e.g. one that needs the XIAO C3): exclude from other envs with `lib_ignore = <libname>` in that env or board section (`<libname>` is a placeholder; no current lib needs this).
- Libs are compiled per env, so a lib must compile for every board whose program includes it; Uno/ATtiny85 have tiny RAM.
- Third-party deps: add to `lib_deps` in the board section or env.
- Editing a lib affects all programs using it; rebuild each affected env.

## Adding a program

1. `mkdir programs/<name>`
2. Add `programs/<name>/main.cpp`
3. In `platformio.ini`, add `[env:<name>-<board>]` for each supported board: extend `[board_<board>]`, set `build_src_filter = -<*> +<name/>`

## Adding a board

1. Add `[board_x]` section in `platformio.ini` (board, platform, framework, flags)
2. Add envs `<program>-<x>` for each program that supports it

## Testing

Unit tests in `test/`. Run with `pio test -e test` (native platform, no hardware, no Python). Suites: `test_balance_core`, `test_balance_parity`, `test_balance_stamp`, `test_balance_supervisor`, `test_dxl_units`, `test_hal_iface`, `test_imu_math`, `test_loop_stats`, `test_tilt`, `test_vector_roundtrip`, `test_wheel_pair` (11 suites, 131 `RUN_TEST` cases by count of the macro; a pass count must come from an actual run).

`test/test_balance_parity` compares the C++ core against the committed golden vectors from Python (`test/test_balance_parity/vectors_generated.inc`); it is the parity gate. Native tests and Wokwi do **not** validate balance on hardware, and the native `test` env (gnu++17) does not catch C++11 violations in firmware headers; build a balance env too.

## Serial Debugging

`tilt_servo` logs `[tag] message` lines (for example `[imu] accel=... fused=... goal=... torque=...`) over USB CDC. Use `pio device monitor -e tilt_servo-xiao_c3`. `balance` logs tagged lines over USB CDC: `[bal]` (boot, ARMED, 10 Hz status with pitch, command, state, cause), `[loop]` (1 Hz min/mean/max/over/n period in us), `[imu]`, `[wheel]`, `[safe]` (latch and torque-off status). Use `pio device monitor -e balance-xiao_c3`. Send `x` to latch the kill (no re-arm; reset the board).

## Risks (balance work)

- Soft-float cost on the C3 (no FPU, reportedly) and loop timing/jitter on the real C3 are unmeasured. The Webots simulator itself alternates 16 ms and 32 ms ticks.
- Sim-derived constants (`K`, `THETA_REF = -0.1086 rad`) will not match the real robot.
- The PID sign convention is unverified even in simulation: LQR and PID give opposite signs for the same tilt (freecad-workspace#359); the hardware checklist ([#34](https://github.com/pluto-atom-4/balancing-robot-controller/issues/34)) checks both.
- Dynamixel runaway / torque left on after a fault is a safety risk. The firmware has a tilt cutoff, torque-off-first, a latching supervisor and a serial kill (unverified on hardware); a true MCU hang cannot be handled in software and the BOOT-button hardware kill is not implemented.
- Firmware headers must be C++11-valid (the native gnu++17 tests will not catch a violation); a firmware build is required after any `lib/` edit.
- Cross-repo drift between the generated copies and the Python source of truth. Run the exporter `--check` before trusting the copies.

## Tracking issues

- Stage F parent (Python repo): https://github.com/pluto-atom-4/freecad-workspace/issues/338
- Parent of parent (Webots simulation): https://github.com/pluto-atom-4/freecad-workspace/issues/10
- C++ umbrella (this repo): https://github.com/pluto-atom-4/balancing-robot-controller/issues/22, children #23–#35 (toolchain/envs, `hal_iface`, `balance_core`, `loop_stats`, `dxl_units`, gains import, parity test, MPU-6050 backend, Dynamixel wheel backend, control loop, Wokwi, hardware checklist, docs), #42 (vector round trip), #43 (`c3_probe`, built, not yet run on hardware), #44 (`c3_facts`, no code yet), #46 (retire legacy envs), #56 (C++11-valid `balance_stamp.h`)
- Python exporter: https://github.com/pluto-atom-4/freecad-workspace/issues/353

**Verification status of these docs:** commands were checked statically against `platformio.ini` and `Makefile`. Commands actually run for the last docs update: `pio test -e test` (131 cases, all passed); `pio run -e balance-xiao_c3`, `pio run -e balance-xiao_c3-pid`, `pio run -e tilt_servo-xiao_c3` and `pio run -e blink-xiao_c3` (all succeeded); and a verbose build of `balance-xiao_c3` to confirm that both `-std=gnu++11` and `-std=gnu++17` appear in the C3 compile line. Not re-run for this update: `make sim-balance-xiao_c3` (last run for #33, PR #55) and the exporter `--check` (last run for #29, PR #51). Any command not listed was not run; nothing was run on real hardware.

**Honesty rule:** nothing in the balance firmware is validated on real hardware. Do not write that a test passed unless it was run, and do not describe a planned feature as implemented.

<!-- code-review-graph MCP tools -->
## MCP Tools: code-review-graph

**This project has a knowledge graph. Start with the code-review-graph
MCP tools to narrow scope, then read the source.** The graph is cheaper than scanning files and
gives you structural context (callers, dependents, test coverage) that file search cannot.

### When to use graph tools FIRST

- **Exploring code**: `semantic_search_nodes_tool` or `query_graph_tool` instead of Grep
- **Understanding impact**: `get_impact_radius_tool` instead of manually tracing imports
- **Code review**: `detect_changes_tool` + `get_review_context_tool` instead of reading entire files
- **Finding relationships**: `query_graph_tool` with callers_of/callees_of/imports_of/tests_for
- **Architecture questions**: `get_architecture_overview_tool` + `list_communities_tool`

### Verify in the source

- Narrow scope with the graph, then read the source. Do not change code from graph output alone.
- For any non-trivial change, read the implementation and the relevant tests before concluding.
- Verify the exact source when touching behavior, database logic, migrations, retries, fallbacks,
  recovery, or compatibility code.
- When the graph and the source disagree, the source wins. The graph may be stale or may not
  model that relationship.
- An empty graph result can mean "not indexed" or "not statically visible", not "does not exist".

### Key Tools

| Tool | Use when |
| ------ | ---------- |
| `detect_changes_tool` | Reviewing code changes — gives risk-scored analysis |
| `get_review_context_tool` | Need source snippets for review — token-efficient |
| `get_impact_radius_tool` | Understanding blast radius of a change |
| `get_affected_flows_tool` | Finding which execution paths are impacted |
| `query_graph_tool` | Tracing callers, callees, imports, tests, dependencies |
| `semantic_search_nodes_tool` | Finding functions/classes by name or keyword |
| `get_architecture_overview_tool` | Understanding high-level codebase structure |
| `refactor_tool` | Planning renames, finding dead code |

### Workflow

1. The graph auto-updates on file changes (via hooks).
2. Use `detect_changes_tool` for code review.
3. Use `get_affected_flows_tool` to understand impact.
4. Use `query_graph_tool` pattern="tests_for" to check coverage.
<!-- /code-review-graph MCP tools -->

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).

## Graph tool routing: Graphify vs. code-review-graph

Macro to micro: start with **graphify** (`GRAPH_REPORT.md`, `graphify query/path/explain`) to place a change in the overall architecture, then switch to **code-review-graph** MCP tools (`query_graph_tool`, `get_impact_radius_tool`, `detect_changes_tool`) to trace the exact structural call paths and blast radius inside that area.

- Don't query both engines in the same reasoning step — escalate from Graphify's macro view down to code-review-graph's micro view only once the relevant subsystem is identified.
- If a graph tool errors, returns empty, or the graph looks stale (`.code-review-graph/graph.db`, `graphify-out/graph.json`), fall back to a narrow, targeted Grep before asking the user — don't retry the same query against the other engine.
- Both graphs update automatically: code-review-graph via the PostToolUse hook after Edit/Write and the pre-commit hook; graphify via `graphify update .` and the PreToolUse search guard. Run `code-review-graph build` / `graphify update .` by hand after a large structural refactor.
