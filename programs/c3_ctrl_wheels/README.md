# c3_ctrl_wheels

Purpose: sim-only Wokwi program for the twin-wheel balancing robot simulator (XIAO C3, two Dynamixel servo chips with IDs 1 and 2 on one bus). It is the twin of `programs/c3_ctrl_wheel`, which drives a single wheel.

Status: copy in progress, not a twin yet. This directory holds a verbatim copy of the single-wheel program files: `main.cpp`, `wokwi.toml`, `diagram.json`, plus `scenario_ctrl_wheels.yaml`. The copied `main.cpp`, `wokwi.toml` and `diagram.json` still have the single-wheel names, the single-wheel build path `.pio/build/c3_ctrl_wheel-xiao_c3` and RFC2217 port 4001. There is no PlatformIO env or Makefile target `c3_ctrl_wheels-xiao_c3` yet.

Do not run this copy under Wokwi before B05 lands. `make sim-c3_ctrl_wheels-xiao_c3` does not work until B04 (env and Makefile target) and B05 (`wokwi.toml` build path, RFC2217 port 4002) land.

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
