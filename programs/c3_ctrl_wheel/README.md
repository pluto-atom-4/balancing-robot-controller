# c3_ctrl_wheel

A sim-only Wokwi program for the XIAO ESP32-C3. The planned firmware will print one per-tick `WHL` wheel-velocity line for the freecad-workspace bridge `poc/ctrl-wheel`; the `WHL` line is not emitted yet.

## Status

- Files are copies of `programs/ttl_servo`. `main.cpp` is still identical to `programs/ttl_servo/main.cpp`; later tasks (#244-#246) strip it and add the `WHL` line.
- Env `c3_ctrl_wheel-xiao_c3` exists; its firmware build succeeded once (the unmodified copy).
- Nothing has been run in Wokwi from this directory.
- Simulation-only. Not hardware-validated. No balance claim.

## Links

- Plan discussion: pluto-atom-4/balancing-robot-controller#234 (https://github.com/pluto-atom-4/balancing-robot-controller/discussions/234)
- Tracking issue for this directory: pluto-atom-4/balancing-robot-controller#235 (https://github.com/pluto-atom-4/balancing-robot-controller/issues/235)
- Tracking issue (T3, README): pluto-atom-4/balancing-robot-controller#237 (https://github.com/pluto-atom-4/balancing-robot-controller/issues/237)
- Parent: pluto-atom-4/freecad-workspace#446 (https://github.com/pluto-atom-4/freecad-workspace/issues/446)
- Counterpart: freecad-workspace `poc/ctrl-wheel` (Python repo, sim-only bridge side)

## Files

- `main.cpp`: copy of `programs/ttl_servo/main.cpp`, `WHL` line not yet added
- `wokwi.toml`: copy of `programs/ttl_servo/wokwi.toml`
- `diagram.json`: copy of `programs/ttl_servo/diagram.json`
- `scenario_ctrl_wheel.yaml`: copy of the ttl_servo scenario file
