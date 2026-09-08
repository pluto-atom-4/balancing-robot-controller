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
