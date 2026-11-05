# c3_ctrl_wheels

Purpose: sim-only Wokwi program for the twin-wheel balancing robot simulator (XIAO C3). The diagram has both Dynamixel servo chips (IDs 1 and 2) on one bus, and the firmware drives both: ID 1 (left, W[0]) and ID 2 (right, W[1], unconfirmed). It is the twin of `programs/c3_ctrl_wheel`, which drives a single wheel.

Status: twin firmware on main (B10 to B13 merged): bring-up of both IDs, one simulated plant per wheel, a WHL2 line per tick, and 10 Hz read telemetry for both IDs. The two-read telemetry of #282 has not been run in a simulator. `diagram.json` has two servo chips, `servo1` (servoId "1") and `servo2` (servoId "2"), plus the FE-URT-1 adapter, all on one DATA net. Log tags are `[ttl_servo]`, `[dxl]`, `[safe]` and `[ctrl_wheel]`, kept from the single-wheel copy. `wokwi.toml` points at the twin build path `.pio/build/c3_ctrl_wheels-xiao_c3` and sets RFC2217 port 4002. The scenario file `scenario_ctrl_wheels.yaml` is adapted from the single-wheel scenario with names updated. Sim-only; nothing is run on hardware.

The RFC2217 ports differ: 4000 (`programs/ttl_servo`), 4001 (`programs/c3_ctrl_wheel`) and 4002 (this program). Running all three at once was not tried.

B04 (#289, env and Makefile target) and B05 (#290, `wokwi.toml`) are merged, so `make sim-c3_ctrl_wheels-xiao_c3` exists.

A scratch merge of B04 (#289), B05 (#290) and B08 (#291) built with `pio run -e c3_ctrl_wheels-xiao_c3` and ran headless: `[dxl] ping ok` and 599 `WHL` lines. B08 only adds declarations that nothing reads yet (the B08 diff is insert-only; the first WHL line matched the single-wheel program's), so this is the still single-wheel behaviour. B04 plus B05 alone were not run together.

A scratch merge of this diagram (B07, commit a6de555) and B09 (#293, its first commit 818273e) built and ran headless with both servo chips on the DATA net and the still single-wheel firmware: `[dxl] ping ok`, 599 `WHL` lines, tx_ok=98, tx_fail=0, rd_ok=110, rd_fail=0. That output is identical, line for line at the end, to the one-servo run above: the simulation is deterministic and the firmware still addresses only ID 1, so this shows that adding a second chip with ID 2 does not disturb ID 1; it does not show servo2 responding. Earlier scratch tests showed that the `servoId` attribute changes the chip's ID (with `servoId` "2" the single-wheel ping to ID 1 failed; with "300" the out-of-range fallback line printed). Not run: B07 alone without B09; later B09 commits (built, not run in a two-chip run); the firmware addressing ID 2 (arrives with B10); a broadcast packet; the Wokwi GUI layout.

The RFC2217 port 4002 was not exercised in the headless runs above.

Not run on hardware; the headless Wokwi runs mentioned above were scratch runs, not CI. Nothing is validated on hardware. Simulation-only.

Plan: https://github.com/pluto-atom-4/balancing-robot-controller/discussions/266

Tracking: issue #267 (umbrella), issue #270 (this copy).

Still planned, not present yet (task IDs from the plan and tracking issue #267):
- B09 (#293, merged): helpers.
- `docs/CTRL_WHEELS.md`

B10 to B13 (bring-up, loop, WHL2 emit, two-ID telemetry) are merged and present in `main.cpp`. B14 (#283) is the header and comment pass on `main.cpp`.

The single-wheel `programs/c3_ctrl_wheel` is not changed by this copy.
