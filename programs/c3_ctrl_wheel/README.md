# c3_ctrl_wheel

A sim-only Wokwi program for the XIAO ESP32-C3. The firmware prints one `WHL` wheel-velocity line per 20 ms tick for the freecad-workspace bridge `poc/ctrl-wheel`, plus a 1 Hz `[ctrl_wheel]` counters line; details in `docs/CTRL_WHEEL.md`.

## Status

- `main.cpp` is the `programs/ttl_servo` copy with the IMU/DOF/EXT_CMD/KILL_POC/HORN_SELFTEST blocks stripped (FALL_TEST kept) and a `WHL` line emitted once per 20 ms tick plus a 1 Hz `[ctrl_wheel]` counters line.
- Env `c3_ctrl_wheel-xiao_c3` builds.
- A headless Wokwi run showed the WHL lines (parsed by the freecad-workspace parser) and the counters line; nothing else was run. No RFC2217 read, no Webots, no hardware.
- Simulation-only. Not hardware-validated. No balance claim. Details in `docs/CTRL_WHEEL.md`.

## Links

- Plan discussion: pluto-atom-4/balancing-robot-controller#234 (https://github.com/pluto-atom-4/balancing-robot-controller/discussions/234)
- Tracking issue for this directory: pluto-atom-4/balancing-robot-controller#235 (https://github.com/pluto-atom-4/balancing-robot-controller/issues/235)
- Tracking issue (T3, README): pluto-atom-4/balancing-robot-controller#237 (https://github.com/pluto-atom-4/balancing-robot-controller/issues/237)
- Parent: pluto-atom-4/freecad-workspace#446 (https://github.com/pluto-atom-4/freecad-workspace/issues/446)
- Counterpart: freecad-workspace `poc/ctrl-wheel` (Python repo, sim-only bridge side)

## Files

- `main.cpp`: `programs/ttl_servo/main.cpp` with the blocks stripped and the `WHL` line added (see Status)
- `wokwi.toml`: copy of `programs/ttl_servo/wokwi.toml`
- `diagram.json`: copy of `programs/ttl_servo/diagram.json`
- `scenario_ctrl_wheel.yaml`: copy of the ttl_servo scenario file
