# c3_ctrl_wheels

Purpose: sim-only Wokwi program for the twin-wheel balancing robot simulator (XIAO C3). PLANNED: two Dynamixel servo chips with IDs 1 and 2 on one bus. It is the twin of `programs/c3_ctrl_wheel`, which drives a single wheel.

Status: copy in progress, not a twin yet. Today this directory holds a single-wheel copy with one servo: `diagram.json` has one servo chip, `servo1`, plus the FE-URT-1 adapter chip, and `main.cpp` uses `DXL_ID = 1`. `main.cpp` line 1 still names `c3_ctrl_wheel`. Its log tags are `[ttl_servo]`, `[dxl]`, `[safe]` and `[ctrl_wheel]`. `wokwi.toml` now points at the twin build path `.pio/build/c3_ctrl_wheels-xiao_c3` and sets RFC2217 port 4002. The scenario file `scenario_ctrl_wheels.yaml` is NEW, adapted from the single-wheel scenario with names updated.

The RFC2217 ports differ: 4000 (`programs/ttl_servo`), 4001 (`programs/c3_ctrl_wheel`) and 4002 (this program). Running all three at once was not tried.

`make sim-c3_ctrl_wheels-xiao_c3` needs the env and Makefile target from B04 (#289) together with this `wokwi.toml` from B05 (#290). Once both are merged it builds and runs the still single-wheel copy. Scratch run by the main session, not re-checked by the builder: headless ping ok and 599 `WHL` lines.

Nothing here is built or run. Nothing is validated on hardware. Simulation-only.

Plan: https://github.com/pluto-atom-4/balancing-robot-controller/discussions/266

Tracking: issue #267 (umbrella), issue #270 (this copy).

Still planned, not present yet:
- B07: second servo
- B08 to B14: `main.cpp` rewrite for two wheels
- `docs/CTRL_WHEELS.md`

The single-wheel `programs/c3_ctrl_wheel` is not changed by this copy.
