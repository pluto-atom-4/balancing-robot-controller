# c3_ctrl_wheels

Purpose: sim-only Wokwi program for the twin-wheel balancing robot simulator (XIAO C3). PLANNED: two Dynamixel servo chips with IDs 1 and 2 on one bus. It is the twin of `programs/c3_ctrl_wheel`, which drives a single wheel.

Status: copy in progress, not a twin yet. Today this directory holds a single-wheel copy with one servo: `diagram.json` has one servo chip, `servo1`, plus the FE-URT-1 adapter chip, and `main.cpp` uses `DXL_ID = 1`. `main.cpp` line 1 still names `c3_ctrl_wheel`. Its log tags are `[ttl_servo]`, `[dxl]`, `[safe]` and `[ctrl_wheel]`. `wokwi.toml` still points at the single-wheel build path `.pio/build/c3_ctrl_wheel-xiao_c3` and sets RFC2217 port 4001. The scenario file `scenario_ctrl_wheels.yaml` is NEW, adapted from the single-wheel scenario with names updated.

This copy shares RFC2217 port 4001 and the single-wheel firmware build path with `programs/c3_ctrl_wheel`, so do not run both. Do not run this copy at all before B05.

`make sim-c3_ctrl_wheels-xiao_c3` does not work until B04 (env and Makefile target) and B05 (`wokwi.toml` build path, RFC2217 port 4002) land.

Nothing here is built or run. Nothing is validated on hardware. Simulation-only.

Plan: https://github.com/pluto-atom-4/balancing-robot-controller/discussions/266

Tracking: issue #267 (umbrella), issue #270 (this copy).

Still planned, not present yet:
- B04: env `c3_ctrl_wheels-xiao_c3` and Makefile target
- B05: `wokwi.toml` build path and RFC2217 port 4002
- B07: second servo
- B08 to B14: `main.cpp` rewrite for two wheels
- `docs/CTRL_WHEELS.md`

The single-wheel `programs/c3_ctrl_wheel` is not changed by this copy.
