# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Multi-program, multi-board PlatformIO project. Main program: self-balancing robot firmware (`balance`) on ESP32-S3. Balance hardware: Seeed Studio XIAO S3 Sense MCU, built-in 6-axis IMU, two Feetech STS3032 bus servos controlled via serial protocol.

Supported boards: Arduino Uno R3 ATmega328P (`uno`, atmelavr), Arduino UNO R4 WiFi (`uno_r4_wifi`, renesas-ra), Seeed XIAO ESP32S3 (`xiao_s3`), generic ESP32 dev (`esp32dev`), ATtiny85 (`attiny85`, atmelavr), Seeed XIAO ESP32C3 (`xiao_c3`, espressif32). More boards can be added.

Programs: `balance` (XIAO S3 / ESP32 only, not built for Uno R3, Uno R4 WiFi, or ATtiny85), `blink` (portable example, all boards), `tilt_servo` (XIAO C3 only: MPU6050 tilt drives Dynamixel XL330 via FE-URT-1).

Envs are named `<program>-<board>`: `balance-xiao_s3` (default), `balance-esp32dev`, `blink-uno`, `blink-uno_r4_wifi`, `blink-xiao_s3`, `blink-esp32dev`, `blink-attiny85`, `tilt_servo-xiao_c3`. Native test env: `test`.

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

**Simulate tilt_servo in Wokwi** (no hardware; servo absent so firmware runs read-only)
```bash
make sim          # build tilt_servo-xiao_c3 + wokwi-cli
```
Config lives in `programs/tilt_servo/` (`wokwi.toml`, `diagram.json`), not the repo root. In CLion set Settings > Wokwi Simulator > config path to `programs/tilt_servo/wokwi.toml`. Run `pio run -e tilt_servo-xiao_c3` first so `.pio/build/tilt_servo-xiao_c3/firmware.{bin,elf}` exist.

**Clean build**
```bash
pio run -t clean
```

## Architecture

### Core Layers (planned shared libs; balance is first consumer)

1. **Servo Control Layer** (`lib/servo/`)
   - STS3032 bus servo protocol implementation
   - Position/speed/torque commands
   - Multi-servo coordination (left/right balance)

2. **IMU/Sensor Layer** (`lib/imu/`)
   - XIAO S3 built-in accelerometer & gyroscope
   - Real-time pitch/roll angle estimation
   - Sensor fusion if needed

3. **Control Algorithm** (`lib/controller/`)
   - PID loops for balance (tilt stabilization)
   - Motor speed regulation
   - Feedback integration

4. **Main Loop** (`programs/balance/main.cpp`)
   - Read IMU → compute error → send servo commands
   - Runs at ~100Hz (typical for balance control)

### File Structure

- `programs/<name>/main.cpp` – One entry point per program (`src_dir = programs`)
  - `programs/balance/main.cpp` – Balance robot control loop (XIAO S3 only)
  - `programs/blink/main.cpp` – Portable blink example
- `include/` – Public headers
- `lib/` – Shared libraries, usable by every program in `programs/` (not tied to one). Each lib is its own folder `lib/<name>/` with `<name>.h` + `<name>.cpp` (optional `library.json`). Currently planned: `servo`, `imu`, `controller` (first user: balance).
- `test/` – Unit tests
- `platformio.ini` – Abstract board sections `[board_uno]`, `[board_uno_r4_wifi]`, `[board_xiao_s3]`, `[board_esp32dev]`, `[board_attiny85]` (board, platform, flags); envs `[env:<program>-<board>]` combine a board section with a program via `build_src_filter = -<*> +<program/>`

## Key Constants/Configs (balance program, XIAO S3)

- **Serial Baud**: 115200 (monitoring & servo communication)
- **Target Loop Freq**: ~100Hz (10ms per cycle)
- **Servo ID Range**: 1–2 (left/right motors)
- **STS3032 Limits**: Position 0–4095 (270°), Speed 1–2047 units

## Build Notes

- `src_dir = programs`; each env selects its program via `build_src_filter`
- `xiao_s3`: board `seeed_xiao_esp32s3` (espressif32), upload speed 921600 baud
- `esp32dev`: generic ESP32 dev board (espressif32)
- ESP32 boards: C++17 (`gnu++17`)
- `uno`: Arduino Uno R3 ATmega328P (atmelavr), no C++17 flag; 16MHz AVR, 2KB RAM, keep sketches small
- `uno_r4_wifi`: Arduino UNO R4 WiFi (renesas-ra, Renesas RA4M1, 32-bit ARM, native USB serial); different chip and uploader than Uno R3 — `blink-uno` on an R4 fails with `stk500_getsync ... not in sync`
- `attiny85`: ATtiny85 (atmelavr), 8MHz internal, 8KB flash, 512B RAM, no hardware UART so no Serial/monitor (blink guards Serial with `__AVR_ATtiny85__`), uploads via ISP programmer (`upload_protocol = usbasp` default in platformio.ini, change to match hardware)
- `balance` not built for `uno`, `uno_r4_wifi`, or `attiny85`
- No external dependencies yet (add as needed)

## Common Edits

- **Tune PID gains**: Look for `Kp`, `Ki`, `Kd` constants in `lib/controller/`
- **Change balance setpoint**: Modify target angle in `programs/balance/main.cpp`
- **Add sensor filtering**: IMU data is noisy; consider complementary/Kalman filter (`lib/imu/`)
- **Servo command mapping**: Servo protocol commands in `lib/servo/STS3032.cpp`
- **Shared lib change**: Edits in `lib/` affect all programs; rebuild each env

## Shared libraries (lib/)

- Any program uses a lib by `#include <name.h>` (or `"name.h"`); PlatformIO finds it automatically (LDF), and only libs actually included are linked, so unused libs cost nothing.
- Keep libs hardware-neutral (no board-specific pins/APIs), or guard with `#if defined(ARDUINO_ARCH_AVR)` / `ARDUINO_ARCH_ESP32` / `ARDUINO_ARCH_RENESAS_UNO` / `__AVR_ATtiny85__`.
- Board-limited lib (e.g. `imu` needs XIAO S3): exclude from other envs with `lib_ignore = imu` in that env or board section.
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

Unit tests in `test/`. Run with `pio test -e test`. Native platform used for desktop testing of control algorithms.

## Serial Debugging

Balance program: servo commands and IMU readings logged to Serial. Use `pio device monitor -e balance-xiao_s3` to inspect. Format: `[tag] message` for structured output.

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
