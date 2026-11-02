# c3_ctrl_wheel

Planned: a sim-only Wokwi program for the XIAO ESP32-C3. Its firmware will print one per-tick `WHL` wheel-velocity line for the freecad-workspace bridge `poc/ctrl-wheel`.

## Status

- Planned, not built.
- Simulation-only. Not hardware-validated.
- Nothing in this directory has been run.
- There is no `main.cpp` yet, so no PlatformIO env builds this program.

## Links

- Plan discussion: pluto-atom-4/balancing-robot-controller#234 (https://github.com/pluto-atom-4/balancing-robot-controller/discussions/234)
- Tracking issue for this directory: pluto-atom-4/balancing-robot-controller#235 (https://github.com/pluto-atom-4/balancing-robot-controller/issues/235)
- Counterpart: freecad-workspace `poc/ctrl-wheel` (Python repo, sim-only bridge side)

## Planned files

- `main.cpp`
- `wokwi.toml`
- `diagram.json`
- `scenario_ctrl_wheel.yaml`

The last three are to be copied and edited from `programs/ttl_servo` by later tasks. None of them exist yet.
