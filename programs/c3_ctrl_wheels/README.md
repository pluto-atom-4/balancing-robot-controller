# c3_ctrl_wheels

Purpose: sim-only Wokwi program for the twin-wheel balancing robot simulator (XIAO C3). The diagram has both Dynamixel servo chips (IDs 1 and 2) on one bus; the firmware drives only ID 1 so far. It is the twin of `programs/c3_ctrl_wheel`, which drives a single wheel.

Status: copy in progress, not a twin yet. Today this directory holds a single-wheel copy: `diagram.json` has two servo chips, `servo1` (servoId "1") and `servo2` (servoId "2"), plus the FE-URT-1 adapter, all on one DATA net; `main.cpp` still drives only `DXL_ID = 1` (single wheel). `main.cpp` line 1 still names `c3_ctrl_wheel`. Its log tags are `[ttl_servo]`, `[dxl]`, `[safe]` and `[ctrl_wheel]`. `wokwi.toml` now points at the twin build path `.pio/build/c3_ctrl_wheels-xiao_c3` and sets RFC2217 port 4002. The scenario file `scenario_ctrl_wheels.yaml` is NEW, adapted from the single-wheel scenario with names updated.

The RFC2217 ports differ: 4000 (`programs/ttl_servo`), 4001 (`programs/c3_ctrl_wheel`) and 4002 (this program). Running all three at once was not tried.

B04 (#289, env and Makefile target) and B05 (#290, `wokwi.toml`) are merged, so `make sim-c3_ctrl_wheels-xiao_c3` exists.

A scratch merge of B04 (#289), B05 (#290) and B08 (#291) built with `pio run -e c3_ctrl_wheels-xiao_c3` and ran headless: `[dxl] ping ok` and 599 `WHL` lines. B08 only adds declarations that nothing reads yet (the B08 diff is insert-only; the first WHL line matched the single-wheel program's), so this is the still single-wheel behaviour. B04 plus B05 alone were not run together.

A scratch merge of B07 (#292) and B09 (#293) at its first commit 818273e built and ran headless with both servo chips on the DATA net and the still single-wheel firmware: `[dxl] ping ok`, 599 `WHL` lines, tx_fail=0, rd_fail=0. Later commits of B09 were built but not run in a two-chip run. Not run: the firmware addressing ID 2 (that arrives with B10), a broadcast packet, the Wokwi GUI layout.

The RFC2217 port 4002 was not exercised in the headless runs above.

Not run on hardware; the headless Wokwi runs mentioned above were scratch runs, not CI. Nothing is validated on hardware. Simulation-only.

Plan: https://github.com/pluto-atom-4/balancing-robot-controller/discussions/266

Tracking: issue #267 (umbrella), issue #270 (this copy).

Still planned, not present yet (task IDs from the plan and tracking issue #267):
- B09 (#293, open): helpers; B10 to B14: bring-up, loop, WHL2 emit, telemetry, header (B08, #291, only adds declarations: constants, a WheelCtx struct and a W[2] array that nothing reads yet)
- `docs/CTRL_WHEELS.md`

The single-wheel `programs/c3_ctrl_wheel` is not changed by this copy.
