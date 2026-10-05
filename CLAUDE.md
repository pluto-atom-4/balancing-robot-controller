# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Multi-program, multi-board PlatformIO project. Main program: self-balancing robot firmware (`balance`). This repo is the **C++ / PlatformIO firmware** side; the sibling Python repo [`pluto-atom-4/freecad-workspace`](https://github.com/pluto-atom-4/freecad-workspace) (project dir `inverted-pendulum-project`) holds the Webots simulation, the Python PID/LQR reference controllers and the Python HAL. Python there, C++ here.

**Status: `programs/balance/main.cpp` is still a stub** (prints an init line, no control). The balance design below is **planned** under umbrella issue [#22](https://github.com/pluto-atom-4/balancing-robot-controller/issues/22) and nothing in it is validated on real hardware. Only `tilt_servo` (bench demo) and `blink` are working programs, and `tilt_servo` has not been validated on hardware either.

### Target hardware (balance, planned)

- **Board: Seeed XIAO ESP32-C3 only.** The ESP32-S3 is a *future enhancement* built on the C3 base. Do not add multi-board support or S3 branches for `balance`.
- **IMU**: MPU-6050 over I2C (the C3 has no built-in IMU). I2C SDA=D4 (GPIO6), SCL=D5 (GPIO7), as in `programs/tilt_servo/main.cpp`.
- **Wheel servos**: Dynamixel **XL330 on the bench now, Feetech STS3032 planned for the final robot.** The motor backend must hide the servo family behind an interface so one can replace the other. The servo bus uses `Serial1` on GPIO20 (RX, D7) / GPIO21 (TX, D6) through an FE-URT-1 adapter, as `tilt_servo` does today. Dynamixel2Arduino is proven to build on the C3 in `tilt_servo`; an STS3032 driver on the C3 is **untested**, and its final design is an open decision. The Dynamixel-specific issues [#27](https://github.com/pluto-atom-4/balancing-robot-controller/issues/27) and [#31](https://github.com/pluto-atom-4/balancing-robot-controller/issues/31) are pending a revision for the XL330-now / STS3032-later split.
- Both PID and LQR are supported, switchable at compile time (planned).

The older text "balance on XIAO S3 Sense with built-in IMU and two STS3032 servos" is stale. The legacy `balance-xiao_s3` and `balance-esp32dev` stub envs still exist; whether to retire them is **pending the human's OK** (issue [#23](https://github.com/pluto-atom-4/balancing-robot-controller/issues/23)).

Supported boards: Arduino Uno R3 ATmega328P (`uno`, atmelavr), Arduino UNO R4 WiFi (`uno_r4_wifi`, renesas-ra), Seeed XIAO ESP32S3 (`xiao_s3`), generic ESP32 dev (`esp32dev`), ATtiny85 (`attiny85`, atmelavr), Seeed XIAO ESP32C3 (`xiao_c3`, espressif32). More boards can be added.

Programs: `balance` (stub today; currently built only for the legacy `xiao_s3` / `esp32dev` envs, not for Uno R3, Uno R4 WiFi, or ATtiny85; target board is the XIAO C3, planned), `blink` (portable example, all boards), `tilt_servo` (XIAO C3 only: MPU6050 tilt drives Dynamixel XL330 via FE-URT-1).

Envs that exist today, named `<program>-<board>`: `balance-xiao_s3` (default, stub), `balance-esp32dev` (stub), `blink-uno`, `blink-uno_r4_wifi`, `blink-xiao_s3`, `blink-esp32dev`, `blink-xiao_c3`, `blink-attiny85`, `tilt_servo-xiao_c3`. Native test env: `test`.

Planned envs (not in `platformio.ini` yet, issue [#23](https://github.com/pluto-atom-4/balancing-robot-controller/issues/23)): `balance-xiao_c3` (LQR default, `-D BALANCE_CONTROLLER_LQR`) and `balance-xiao_c3-pid` (PID, `-D BALANCE_CONTROLLER_PID`); `main.cpp` should fail with `#error` if neither flag is set. #23 checks whether the hyphenated env name works with the Makefile and may rename it, so do not hard-code the name elsewhere until #23 lands. The test env is also planned to get `-Wall -Wextra -ffp-contract=off`.

## Development Commands

Use `-e <program>-<board>` to pick an env. Default env: `balance-xiao_s3`.

**Build default env** (balance-xiao_s3)
```bash
pio run
```

**Build one env**
```bash
pio run -e balance-xiao_s3
pio run -e blink-uno
pio run -e blink-attiny85
```

**Upload one env**
```bash
pio run -e blink-uno -t upload
pio run -e blink-attiny85 -t upload
pio run -e balance-xiao_s3 -t upload
```

**Monitor serial output** (115200 baud)
```bash
pio device monitor -e blink-uno
```

**Run unit tests**
```bash
pio test
```

**Simulate in Wokwi** (no hardware; needs `wokwi-cli` or the CLion Wokwi plugin)
```bash
make sim                       # tilt_servo-xiao_c3 (servo absent, firmware runs read-only)
make sim-blink-xiao_c3         # blink control sketch, serial-only check of the sim setup
make sim-<program>-<board>     # any env whose programs/<program>/ has wokwi.toml + diagram.json
```
Each program keeps its own config in `programs/<program>/` (`wokwi.toml`, `diagram.json`), paths relative to the toml. `make sim-<env>` builds the env, then runs `wokwi-cli programs/<program>`. In CLion set Settings > Wokwi Simulator > config path to that program's `wokwi.toml`. Pass extra CLI flags with `WOKWI_ARGS="--timeout 10000"`. The C3 firmware uses USB CDC serial, so each C3 `diagram.json` must set `"serialInterface": "USB_SERIAL_JTAG"` on the board part or no serial output shows. The CLI never exits on its own (firmware loops), so `sim` ends with the timeout exit code 42.

**Clean build**
```bash
pio run -t clean
```

## Architecture

### Planned balance layout (planned; none of these libs exist yet)

Replaces the older `lib/servo`, `lib/imu`, `lib/controller` sketch. Headers use the `.h` extension, matching `lib/tilt/tilt.h`.

- `lib/hal_iface/hal_iface.h` – C++ mirror of the Python HAL contract (`kContractVersion`).
- `lib/balance_core/balance_core.h` – header-only float32 PID + LQR core, no heap. Gains are passed to the constructors, so the core does not depend on the generated header.
- `lib/loop_stats/` – loop period min/mean/max tracker (pure, natively tested).
- `lib/dxl_units/` – rad/s <-> Dynamixel raw velocity conversion (pure, natively tested). Its constants are UNVERIFIED against the XL330 e-Manual; the tracking issue requires a citation.
- `programs/balance/` – Arduino-bound code only: `imu_mpu6050.h`, `dxl_wheels.h`, `config.h`, `main.cpp`.

### Project split and generated files (planned)

- **Python is the source of truth** for controller gains and golden parity vectors. This repo keeps committed, **generated** copies. **Do not hand-edit generated files**; regenerate them from the Python repo and commit the result.
- Planned generated files (not present yet):
  - `lib/balance_core/balance_gains_generated.h` – LQR gain K, resting-pitch offset `THETA_REF`, output limit, PID gains, plus a schema / contract-version / gains-hash stamp.
  - `test/test_balance_parity/vectors_generated.inc` – golden test vectors as plain C arrays, so native Unity tests need no Python or scipy.
- The Python exporter is planned as `export_cpp.py` ([freecad-workspace#353](https://github.com/pluto-atom-4/freecad-workspace/issues/353)), run manually against a local checkout of this repo, with a `--check` drift detector. There is **no cross-repo CI**. Workflow: change gains or vectors in Python, run the exporter, commit the regenerated copies here.
- The HAL contract is **hand-mirrored**: Python has `HAL_CONTRACT_VERSION`, this repo's planned `hal_iface.h` carries `kContractVersion`. Any change to units (rad, rad/s), pitch sign, `THETA_REF`, or dt semantics must bump the version in both repos.
- dt semantics (contract v1): `dt == 0` means P-only; `dt < 0` means output 0 plus a fault flag; `wait_next_tick` returning `< 0` means stop.

### Diagnostic / probe programs (planned)

Planned as permanent diagnostic tools, each with its own env, as new children of #22 (issues not filed yet, so no links):

- `c3_probe` – times the pieces of a balance loop on the real C3 (IMU read, servo write, soft-float PID/LQR step cost, free heap, loop period).
- `c3_facts` – static IMU facts (gyro bias in rad/s, resting pitch and its sign) and servo model, limits and a velocity-unit ramp. The ramp is capped around 20 % of the velocity limit, about 15 s, then torque off, with the servo off the robot and held down.
- A native vector round-trip test (`pio test -e test`, no hardware) is also proposed.

None of these exist yet, so there is no run procedure to document.

### File Structure (what exists today)

- `programs/<name>/main.cpp` – One entry point per program (`src_dir = programs`)
  - `programs/balance/main.cpp` – Balance stub (init message only); real control loop planned (#22)
  - `programs/blink/main.cpp` – Portable blink example
  - `programs/tilt_servo/main.cpp` – MPU6050 tilt drives Dynamixel XL330 (XIAO C3)
  - `programs/<name>/wokwi.toml` + `diagram.json` – Wokwi sim config (`blink`, `tilt_servo`)
- `include/` – Public headers
- `lib/` – Shared libraries, usable by every program in `programs/` (not tied to one). Each lib is its own folder `lib/<name>/` with `<name>.h` (+ `<name>.cpp` if needed; optional `library.json`). Exists today: `lib/tilt/tilt.h` (header-only pitch math, used by `tilt_servo`). The planned balance libs are listed above.
- `test/` – Unit tests (today: `test/test_tilt`)
- `platformio.ini` – Abstract board sections `[board_uno]`, `[board_uno_r4_wifi]`, `[board_xiao_s3]`, `[board_xiao_c3]`, `[board_esp32dev]`, `[board_attiny85]` (board, platform, flags); envs `[env:<program>-<board>]` combine a board section with a program via `build_src_filter = -<*> +<program/>`

## Key Constants/Configs

`tilt_servo` (XIAO C3, exists today; values from `programs/tilt_servo/main.cpp`):

- **Serial (USB CDC)**: 115200, monitoring only. `ARDUINO_USB_CDC_ON_BOOT=1` keeps `Serial` off GPIO20/21.
- **Servo bus**: `Serial1`, GPIO20 (RX) / GPIO21 (TX), 57600 baud, Dynamixel protocol 2.0, ID 1, through FE-URT-1 (direction handled in hardware, `DXL_DIR_PIN = -1`).
- **I2C**: SDA=D4 (GPIO6), SCL=D5 (GPIO7), 400 kHz.
- **Loop**: 50 Hz.

`balance` targets (planned, all unmeasured on hardware): loop rate, jitter and soft-float cost on the C3 are to be measured by the planned `c3_probe`. Do not assume the ~100 Hz mentioned in older notes.

Legacy (previous S3 plan, **not** part of the C3 plan, unverified): two STS3032 servos, IDs 1–2, position 0–4095 (270°), speed 1–2047 units. The STS3032 is the planned servo for the final robot; its C3 driver is untested.

## Build Notes

- `src_dir = programs`; each env selects its program via `build_src_filter`
- `xiao_s3`: board `seeed_xiao_esp32s3` (espressif32), upload speed 921600 baud
- `xiao_c3`: board `seeed_xiao_esp32c3` (espressif32, RISC-V), upload speed 921600 baud, `ARDUINO_USB_MODE=1` + `ARDUINO_USB_CDC_ON_BOOT=1` (Serial over native USB). The target board for `balance`. The C3 reportedly has no hardware FPU, so float32 would be soft-float; this is **not yet verified** (to be checked in #23) and its cost is unmeasured.
- `esp32dev`: generic ESP32 dev board (espressif32)
- ESP32 boards: C++17 (`gnu++17`)
- `uno`: Arduino Uno R3 ATmega328P (atmelavr), no C++17 flag; 16MHz AVR, 2KB RAM, keep sketches small
- `uno_r4_wifi`: Arduino UNO R4 WiFi (renesas-ra, Renesas RA4M1, 32-bit ARM, native USB serial); different chip and uploader than Uno R3 — `blink-uno` on an R4 fails with `stk500_getsync ... not in sync`
- `attiny85`: ATtiny85 (atmelavr), 8MHz internal, 8KB flash, 512B RAM, no hardware UART so no Serial/monitor (blink guards Serial with `__AVR_ATtiny85__`), uploads via ISP programmer (`upload_protocol = usbasp` default in platformio.ini, change to match hardware)
- `balance` not built for `uno`, `uno_r4_wifi`, or `attiny85`
- External dependencies are pinned per env in `platformio.ini` (today only `tilt_servo-xiao_c3`: Adafruit MPU6050 / Unified Sensor / BusIO / GFX / SSD1306 and Dynamixel2Arduino)

## Common Edits

- **Tune PID/LQR gains** (planned): do **not** edit `balance_gains_generated.h` by hand. Change the gains in the Python repo and regenerate (see "Project split and generated files").
- **Change balance setpoint** (planned): the resting-pitch offset `THETA_REF` is also generated from Python. The sim value (`-0.1086 rad`) will not match the real robot; the real equilibrium angle must be measured.
- **Tilt/IMU math today**: `lib/tilt/tilt.h` (complementary filter, pitch, tick conversion) and `programs/tilt_servo/main.cpp`.
- **Servo command mapping**: Dynamixel calls are in `programs/tilt_servo/main.cpp` (Dynamixel2Arduino). The planned servo-family interface (XL330 now, STS3032 later) does not exist yet.
- **Shared lib change**: Edits in `lib/` affect all programs; rebuild each env

## Shared libraries (lib/)

- Any program uses a lib by `#include <name.h>` (or `"name.h"`); PlatformIO finds it automatically (LDF), and only libs actually included are linked, so unused libs cost nothing.
- Keep libs hardware-neutral (no board-specific pins/APIs), or guard with `#if defined(ARDUINO_ARCH_AVR)` / `ARDUINO_ARCH_ESP32` / `ARDUINO_ARCH_RENESAS_UNO` / `__AVR_ATtiny85__`.
- Board-limited lib (e.g. one that needs the XIAO C3): exclude from other envs with `lib_ignore = imu` in that env or board section.
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

Unit tests in `test/`. Run with `pio test -e test`. Native platform used for desktop testing of control algorithms. Today the only suite is `test/test_tilt`.

Planned (not present yet): a native Unity parity test, `test/test_balance_parity`, comparing the C++ core against the committed golden vectors from Python (`vectors_generated.inc`). It is intended as the parity gate (`pio test -e test`, no hardware, no Python needed). Native tests and Wokwi do **not** validate balance on hardware.

## Serial Debugging

`tilt_servo` logs `[tag] message` lines (for example `[imu] accel=... fused=... goal=... torque=...`) over USB CDC. Use `pio device monitor -e tilt_servo-xiao_c3`. The `balance` stub only prints an init line; its planned logging is not defined yet.

## Risks (planned balance work)

- Soft-float cost on the C3 (no FPU, reportedly) and loop timing/jitter on the real C3 are unmeasured. The Webots simulator itself alternates 16 ms and 32 ms ticks.
- Sim-derived constants (`K`, `THETA_REF = -0.1086 rad`) will not match the real robot.
- The PID sign convention is unverified even in simulation; the hardware checklist ([#34](https://github.com/pluto-atom-4/balancing-robot-controller/issues/34)) checks it.
- Dynamixel runaway / torque left on after a fault is a safety risk; the firmware needs a tilt cutoff and fail-safe.
- Cross-repo drift between the generated copies and the Python source of truth.

## Tracking issues

- Stage F parent (Python repo): https://github.com/pluto-atom-4/freecad-workspace/issues/338
- Parent of parent (Webots simulation): https://github.com/pluto-atom-4/freecad-workspace/issues/10
- C++ umbrella (this repo): https://github.com/pluto-atom-4/balancing-robot-controller/issues/22, children #23–#35 (toolchain/envs, `hal_iface`, `balance_core`, `loop_stats`, `dxl_units`, gains import, parity test, MPU-6050 backend, Dynamixel wheel backend, control loop, Wokwi, hardware checklist, docs)
- Python exporter (planned): https://github.com/pluto-atom-4/freecad-workspace/issues/353

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
