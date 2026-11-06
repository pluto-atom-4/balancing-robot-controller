# c3_ctrl_wheels: twin-wheel WHL2 relay (sim-only)

> **Status: SIM-ONLY, simulation-only, not hardware-validated.** This is a RELAY, not a closed loop:
> the firmware prints WHL2 lines and receives nothing back from Webots. No balance claim. The wheel
> chip-side signs are UNCONFIRMED (with the bridge flags and the Webots controller constants at their defaults +1/+1, the net direction was owner-observed to roll both wheels toward +x for a positive command, 2026-11-06, row (l); no pass criterion is defined).
> Latency is UNMEASURED. Dated 2026-11-05, Webots result added 2026-11-06.

FW is freecad-workspace, the Python sim repo (Webots, bridge). BRC is this repo. The single-wheel
sibling doc is [CTRL_WHEEL.md](CTRL_WHEEL.md). The host-side contract is in the FW repo:
[CONTRACT.md](https://github.com/pluto-atom-4/freecad-workspace/blob/main/poc/ctrl-wheels/CONTRACT.md)
and the FW README is
[README.md](https://github.com/pluto-atom-4/freecad-workspace/blob/main/poc/ctrl-wheels/README.md).

## 1. What it is

`programs/c3_ctrl_wheels/` is a sim-only Wokwi program for the XIAO ESP32-C3, env
`c3_ctrl_wheels-xiao_c3` (`platformio.ini`, `build_src_filter = -<*> +<c3_ctrl_wheels/>`). It is
derived from `programs/ttl_servo` and is the twin of `programs/c3_ctrl_wheel`, which drives one wheel.

- Two Dynamixel XL330 sim chips sit on one servo bus through the `wokwi-fe-urt-1` adapter
  (`diagram.json`): `servo1` with servoId 1 (left, `W[0]`) and `servo2` with servoId 2 (right, `W[1]`).
  The ID 2 assignment is unconfirmed (`SERVO_ID_RIGHT` in `main.cpp`).
- Two plants. Each wheel has its own `segway::SegwaySim` (`lib/segway_sim`). Both are ticked with the
  same filtered MPU6050 gyro Z and the same dt.
- One `WHL2` line per 20 ms loop tick (format in section 3), for the FW twin contract (CONTRACT.md S1).
- A 10 Hz telemetry line that reads both IDs (section 6) and a 1 Hz `[ctrl_wheel]` counters line.
- Serial: `Serial1` on GPIO20 (RX) and GPIO21 (TX) at 57600 baud, protocol 2.0, velocity mode, as in
  `ttl_servo`.
- RFC2217 port 4002 (`wokwi.toml`). Port 4000 is `ttl_servo` and 4001 is `c3_ctrl_wheel`.

The scratch README in `programs/c3_ctrl_wheels/README.md` predates this twin firmware. It still
describes a single-wheel copy, so this document follows `main.cpp` instead.

## 2. Values that are never mixed

| Name | What it is | Where it comes from |
| --- | --- | --- |
| `model` | The LEFT plant's lagged wheel velocity (`W[0]`), rad/s. **Not a measurement.** Printed as 0 while holding. Not zeroed in state 2. | `W[0].sim.wheel_vel_rad_s` in `main.cpp` (`whl_model`). |
| `rawL` | The raw last CONFIRMED on the left chip (ID 1), already signed. Unknown reports 0. Reports 0 in state 2. | `last_raw` of `W[0]`: set to the signed value on a successful write and to 0 on a successful zero write. Reset to unknown (INT32_MIN) on a failed write, a failed zero write, any torque-off attempt, and a failed bring-up step. |
| `rawR` | The raw last CONFIRMED on the right chip (ID 2), already signed. Unknown reports 0. Reports 0 in state 2. | `last_raw` of `W[1]`, same rule. |
| Webots angle and velocity | Measurements, host side only. | The FW bridge CSV. **Not emitted by this firmware.** |

Rules for anyone reading the stream:

- Do not call `model` a measured wheel speed, and do not treat it as the right wheel's value.
- Contrast with the single-wheel `docs/CTRL_WHEEL.md` (section 3). There, `raw` is derived from the
  model: `segway::wheel_cmd_to_raw(whl_model)`. In this twin the raws come from the chip-side
  bookkeeping (`last_raw`), not from the model, so they can differ from what the model implies after
  a failed write.
- The Webots values are never in this firmware's output.

## 3. The WHL2 line

Format, one line per tick, terminated by `\n`:

```
WHL2,<seq>,<t_us>,<model %.6f>,<rawL>,<rawR>,<state>*HH
```

The template above is a format, not captured output.

| Field | Type and rule |
| --- | --- |
| `seq` | uint16, printed unsigned. Advances on every `whl_emit` call, including a dropped or refused line, so a gap on the host shows a drop. Wraps 65535 to 0, about 22 min at 50 Hz. |
| `t_us` | uint32, `micros()` at the moment the line is formatted. The emit time, not a sample time. Wraps about every 71.6 min. |
| `model` | `%.6f` of `W[0]`'s plant velocity, or 0 while holding. A non-finite value is refused. A value printed as `-0` drops its sign and prints `0.000000`. |
| `rawL` | Signed decimal (int32), the last value confirmed on the left chip, as written (signed). |
| `rawR` | Signed decimal (int32), the last value confirmed on the right chip, as written (signed). |
| `state` | One decimal digit: `0` run, `1` hold after a fall, `2` bus down. |
| `HH` | Two uppercase hex digits: the XOR of every byte from `W` up to, not including, `*`. |

Refusal (`lib/wheels_frame/wheels_frame.h`, `format`): when the output pointer is null or the capacity
is 0, `format` returns 0 without writing. Otherwise any other refusal (`state` above 2, a non-finite
model, or a formatting step that does not fit in the capacity) sets `out[0]` to NUL and returns 0.
The firmware passes capacity 96. In `whl_emit`, a 0 return is not written and is counted as `dropped`.

Size and emit gate:

- `whl_buf[96]` includes the NUL. 96 is a sizing choice, not measured.
- The size note in `programs/c3_ctrl_wheels/main.cpp` (the comment above `whl_buf`) gives 61 bytes for
  a line with int32 raws and a 9-character model. This is arithmetic from the field widths, not a
  measurement: `WHL2,` 5, seq 5 digits, `,` 1, t_us 10 digits, `,` 1, model 9, `,` 1, rawL 11, `,` 1,
  rawR 11, `,` 1, state 1, `*HH` 3, newline 1. That is 61 including the newline (62 with the NUL).
  A larger `|model|` prints a longer line.
- `whl_emit` writes the line only if `Serial.availableForWrite()` is above 0 and at least the line
  length. Otherwise the line is dropped and counted in `dropped`.
- Whether `Serial.write` can block on the C3 USB CDC is not verified. The other `Serial` prints in
  `loop()` are not gated, so they can block when the host stalls.

Wraps:

- `seq` is printed as an unsigned value and wraps after 65536 emit calls (about 22 min at 50 Hz).
- `t_us` wraps after about 71.6 min. The host must handle both wraps. This repo's tests do not show
  that.

State table:

- `0` run.
- `1` hold after a fall. Set when either plant has fallen, cleared 1 s later.
- `2` bus down (latched). `2` wins over `1` when both are present.

State notes:

- **State 0 does not mean the writes succeeded.** A tick whose write failed, but did not reach 3 in a
  row, keeps state 0.
- **State 2 reports 0 for both raws.** The model is not zeroed in state 2 (only while holding).
- **State 1 (hold).** The model prints 0. The raws print 0 as well: the fall stop sets each `last_raw`
  to 0 when confirmed, or to unknown (printed 0), and no writes happen during the hold. This is read
  from the source. State 1 was observed in section 10 row (f) (model 0, rawL and rawR 0 in the holds).
- **OPEN limitation: the 0 in `rawL` and `rawR` is ambiguous.** It can mean a confirmed zero, an
  unknown chip state (before the first write, after a failed write, or after a torque-off attempt),
  or state 2. A host cannot tell these apart from the line. No code change is planned in this
  document.

Checksum: XOR of the body bytes, same routine as `lib/imu_frame` (reused through `lib/wheels_frame`).

## 4. Sign convention

- The signs are defined only in `programs/c3_ctrl_wheels/main.cpp` (`WHEEL_SIGN_LEFT = 1`,
  `WHEEL_SIGN_RIGHT = -1`), both marked unconfirmed.
- The sign is applied only in `write_goal_raw()`: `wire = sign * raw`. The raw is the output of
  `segway::wheel_cmd_to_raw` (`lib/segway_sim/segway_sim.h`, lines 68-70), which calls
  `dxl_units::radSToRaw`. That value is signed by the plant's wheel velocity and clamped to
  +-`segway::kWheelCapRaw` (125, `segway_sim.h` line 64) before the wheel sign is applied, so the wire
  value stays within +-125. The rounding and clamp are in `lib/dxl_units/dxl_units.h` (lines 37-50).
- The chip therefore receives `rawL = +raw` and `rawR = -raw`. The WHL2 line carries the signed values
  that were confirmed on each chip.
- The model carries no sign. It is the left plant's own velocity.
- There are three sign places: (1) the chip signs above, UNCONFIRMED;
  (2) the bridge `--sign-left` and `--sign-right` flags, default +1 and +1, applied to the model before
  the clamp (the raw columns never reach the command); (3) the Webots controller constants
  `SIGN_LEFT = 1` and `SIGN_RIGHT = 1`. Only the net Webots direction with (2) and (3) at their defaults
  was observed by the owner (row (l)); (2) and (3) were not tested separately. The flags are described in the
  [FW README, Sign convention](https://github.com/pluto-atom-4/freecad-workspace/blob/main/poc/ctrl-wheels/README.md#sign-convention).
- `rawL == -rawR` is expected by construction. Both plants get identical inputs, and the wire value is
  the sign times the same raw. It can differ only after one wheel's write or torque-off has left its
  last-confirmed value out of step with the other's. It is not evidence of two independently behaving
  wheels.
- Webots direction check (owner-observed in the Webots 3D view, 2026-11-06, sim only, row (l)): with the default
  bridge signs +1/+1 a positive command turns both wheels about world +y in the same sense and they roll toward
  +x. In the owner's description, viewed from each wheel's outside they look opposite and viewed from one side the sense is the same (a description, not a separate measurement). This says nothing
  about the chip signs (`rawL = +raw`, `rawR = -raw`), which stay UNCONFIRMED.

## 5. Stop and latch rules

Each rule is marked with its simulator status. "Not run in a simulator" means no run has shown the
path; the owner captures in section 10 cover the normal running path, the fall path (row (f)), the per-tick failure lines only with INJECTED failures (rows (d), (g), (h), (i), (k)), and the three-failed-tick latch only with INJECTED failures (rows (d), (g) and (i)). None of these was run with a non-injected mid-run failure.

- **Bring-up (`setup()`).** For ID 1 then ID 2: ping, torque off, velocity mode, goal 0, torque on.
  It stops at the first failure, so ID 2 is not tried if ID 1 fails. On any failure the firmware sends
  a forced stop to both IDs (zero velocity, then torque off) and latches. The success path is implied
  by the numeric `pos1` and `pos2` in the section 10 (b) run. The ID 2 failure path was observed in section 10 row (j) (missing chip, no injection). The ID 1 failure path was not run in a
  simulator.
- **Per tick (`loop()`).** Both goal writes are always attempted. A tick fails if either write fails.
  A write on change (the signed value equals `last_raw`) makes no bus call and counts as a success.
  The write path is seen in section 10 (state 0). The failure path was seen only with INJECTED failures (rows (d), (g), (h), (i), (k)); not run with a non-injected mid-run failure.
- **Three failed ticks in a row.** A forced torque-off on both IDs (zero velocity first, up to 3
  tries per step), then the latch `bus_down`. The latch is terminal: no Dynamixel calls, reset the
  board to retry. Seen only with INJECTED failures (rows (d), (g) and (i); (g) is uncounted); not run with a non-injected mid-run failure.
- **Fall (either plant `has_fallen`).** A zero-velocity stop on both IDs. Torque off is sent only if a
  zero write failed. If the stop returns false (any zero write failed), the bus latches. Then a 1 s
  hold: the plants are frozen and no writes go out. Then both plants restart and the failure streak
  resets. Observed in section 10 row (f) (no injection): 5 FELL, restarts and ok=1 stops. Not run with a non-injected mid-run failure; the branch where a zero write fails and the bus latches was not run.
- **After a latch.** No goal writes, and no stop is sent on a fall. The hold and restart still run,
  and the state stays 2. Not run in a simulator (source only): no capture shows a fall after a latch.
- **No IMU at boot (`Failed to find MPU6050 chip`).** The firmware pings both IDs (the ping result is
  not printed), sends the forced stop to both IDs without a `bus_down` check, then halts in a loop. It
  does not call `latch_bus_down`. Not run in a simulator.
- **Gyro calibration failure (fewer than 50 of the 100 reads good).** It prints
  `gyro calibration failed (IMU reads), halting`, sends the forced stop only if the bus is not already
  latched, then halts in a loop. It does not call `latch_bus_down`. The loop never prints WHL2 after
  either halt. Not run in a simulator.

Every call to `latch_bus_down()` in `main.cpp` is one of: bring-up failure, three failed ticks, or a
failed stop after a fall. This program has no tilt cutoff, no stall watchdog and no serial kill;
those are features of the balance firmware, not of this program.

## 6. Telemetry

10 Hz line (`PRINT_PERIOD_MS` = 100):

```
[ttl_servo] sim theta=.. rate=.. wheel=.. cmd=.. pos1=.. dpos1=.. pos2=.. dpos2=.. gyroZ=.. bus=..
```

| Key | Meaning |
| --- | --- |
| `cmd` | `W[0]`'s last CONFIRMED signed raw. `NA` when that state is unknown. A confirmed zero prints 0. Left only. |
| `pos1`, `pos2` | Present position read from ID 1 and ID 2. A number when the read succeeded, `ERR` when it failed, `NA` when `servo_ok` is false (bring-up failed or the bus is latched). |
| `dpos1`, `dpos2` | Change since the previous successful read of that ID, computed as `((pos - last + 6144) % 4096) - 2048`, so the range is -2048 to 2047 when both positions are in 0..4095 (the read value is not clamped in the code, so this assumes the chip reports in that range). `n/a` until a previous successful read exists. A failed read clears it, so the next good read prints `n/a`. |
| `bus` | 1 when `servo_ok` (both wheels came up and the bus is not latched), else 0. |
| `wheel` | `W[0]`'s plant velocity, shown as 0 while holding. |

Reads: when `servo_ok` is true, each 10 Hz tick makes two blocking reads (ID 1, then ID 2). Reads
never count toward the write failure or the tick streak, and never latch. On a dead or slow bus the
source comment says a tick can stall at least twice the library read timeout if the library makes one
attempt per call. The attempts per call and the timeout value were not checked. The stall was not
measured. `clamp_dt_s` hides part of it from the sim.

1 Hz bus line (only while `servo_ok`):

```
[dxl] tx_ok=.. tx_fail=.. rd_ok1=.. rd_fail1=.. rd_ok2=.. rd_fail2=.. max_us=..
```

- `tx_ok` and `tx_fail` count goal writes and zero writes, one count per attempt. Torque-off
  transactions are not counted.
- `rd_ok1`, `rd_fail1`, `rd_ok2`, `rd_fail2` count the two reads per tick.
- `max_us` is the longest single transaction (writes and reads) since the previous print. It resets at
  each 1 Hz print. The same line is printed once when the bus latches, without that reset.

Counters line (1 Hz, only when `Serial.availableForWrite()` is at least 80):

```
[ctrl_wheel] sent=.. dropped=.. max_emit_us=..
```

- `sent` and `dropped` are cumulative since boot.
- `max_emit_us` is the longest `whl_emit` call since the previous counters line. It resets on each print.
  The timing is the simulated clock, not a C3 measurement.

Other lines (not exhaustive):

- Bring-up, per ID: `[dxl] ping id=.. ok` or `[dxl] ping id=.. FAILED`; `[dxl] torque_off id=.. ok=..`;
  `[dxl] mode_velocity id=.. ok=..`; `[dxl] goal0 id=.. ok=..`; `[dxl] torque_on id=.. ok=..`;
  `[dxl] bring-up id=.. failed`; and once for the whole bring-up, `[dxl] bring-up failed, bus down, no Dynamixel calls` (printed before the forced stop, which does make Dynamixel calls).
- Boot: `[ttl_servo] gyro Z bias=.. rad/s (keep the slider at 0 during boot)`;
  `[ttl_servo] Failed to find MPU6050 chip`; `[ttl_servo] gyro calibration failed (IMU reads), halting`;
  `[ttl_servo] FALL TEST: kp=2 kd=1 (expect FELL within ~6 s)` (only with `TTL_SERVO_FALL_TEST`).
- Stops and latch: `[dxl] stop id=.. vel=0 ok=..`, `[safe] torque off <why> id=..` (with
  `(TORQUE OFF NOT CONFIRMED)` appended when not confirmed), `[safe] bus down (reset the board to retry)`.
- Fall: `[ttl_servo] FELL id=.. theta=..`, `[ttl_servo] restart`.
- Writes: `[dxl] write failed id=.. raw=.. (n=..)`, logged on the first failed tick and every 50th. The
  printed raw is the argument to `write_goal_raw()`: the `wheel_cmd_to_raw` output, signed and clamped
  to +-125, before the wheel sign is applied. It can be negative.
- `[ttl_servo] IMU read failed, holding last gyro Z`.

Log tags are the ones kept from the single-wheel copy: `[ttl_servo]`, `[dxl]`, `[safe]` and `[ctrl_wheel]`.

## 7. Build and run

```bash
make wokwi-chips                      # builds the shared chips; the sim target depends on them (needs network on first use)
pio run -e c3_ctrl_wheels-xiao_c3     # builds the env; not run for this document
make sim-c3_ctrl_wheels-xiao_c3 WOKWI_ARGS="--scenario scenario_ctrl_wheels.yaml --timeout 40000"   # needs WOKWI_CLI_TOKEN
```

- The headless run never exits by itself. Exit 42 from the timeout is expected.
- `scenario_ctrl_wheels.yaml` was not run. Owner-reported: headless `wokwi-cli` runs hit the Free-plan
  CI minute quota on 2026-11-05. Whether the owner's VS Code captures used this scenario is not stated.
- The scenario waits for `ping id=1 ok` and `ping id=2 ok` (in print order) and then `Sim starts at 15 deg`,
  then runs about 12 s: 4 s wait, 4 s at 30 deg/s, 4 s at 0. The suggested `--timeout` is 40000 ms; the
  reasoning is in the scenario header (`programs/c3_ctrl_wheels/scenario_ctrl_wheels.yaml`). In short: 40000 ms
  is a judgement chosen to leave room, not derived or measured.
  It is not known whether --timeout is wall-clock or sim time, and the simulator ran between 0.63 and 1.00 of host time in the eight captured runs (table in the section 10 note), so it can run slower than real time and the speed differs between runs. Scenario not run. The sim is headless
  only and is not a balance test. A failed or missing wait ends at `--timeout` (exit 42), so exit code 42
  does not prove the pings matched. A deliberately wrong wait-serial string run (recording the real exit code of a failed wait) is planned in V01 (#304), not run.
- The fall path needs `PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_FALL_TEST"` (low gains kp=2, kd=1). Run: section 10 row (f), captured by the owner (see section 10).
- The owner's route is the VS Code Wokwi extension (see the quota note above). Whether the extension
  executes scenario files is not known.
- RFC2217: `wokwi.toml` sets `rfc2217ServerPort = 4002`. Port 4002 has the same exposure as the 4000
  port: plain TCP with no authentication, and its bind address was not checked. The owner acceptance
  recorded in CLAUDE.md (discussion 228) covers 4000 only and is not extended here. The three-listener
  case (4000, 4001 and 4002 at once) was never tried. Whether the VS Code listener stays
  open on 4002 after a stop was not checked (it is observed for 4000 only).
- Bridge read: `--port rfc2217://localhost:4002`. The start order (Webots, bridge, simulation) and the
  other bridge flags are in the
  [FW README](https://github.com/pluto-atom-4/freecad-workspace/blob/main/poc/ctrl-wheels/README.md).
  They are not repeated here.

## 8. Not modeled and not claimed

- Known gaps in the Wokwi model are tracked in X01,
  https://github.com/pluto-atom-4/balancing-robot-controller/issues/286. That issue is the only list
  this document points to. It is not exhaustive, and nothing about the XL330 is restated here from memory.
- Not claimed:
  - a closed loop through Webots (the firmware reads nothing from Webots or the bridge);
  - any balance result or gain check;
  - the chip-side wheel sign against Webots or against the servo (only the net direction with the bridge flags and Webots constants at their defaults +1/+1 is owner-observed, row (l));
  - latency, on the host or on a real bus;
  - independent per-wheel dynamics (both plants share the gyro input and the dt);
  - anything on hardware. Nothing here is hardware-validated.
- The physics constants in `lib/segway_sim` are guesses, so the model values do not describe a real robot.

## 9. Origins

- Derived from `programs/ttl_servo`, and the twin of `programs/c3_ctrl_wheel` (the single-wheel program,
  not changed by this program).
- Reused, unchanged: `lib/segway_sim` (plant and PD), `lib/imu_frame` (through `lib/wheels_frame`,
  which includes it), and the shared chips in `wokwi/chips/` (`wokwi-fe-urt-1`, `wokwi-ttl-servo`).
  `lib/dxl_units` is reached through `lib/segway_sim`: `segway_sim.h` includes `dxl_units.h`, and
  `wheel_cmd_to_raw` calls `dxl_units::radSToRaw`. `main.cpp` includes `segway_sim.h` and
  `wheels_frame.h` directly.
- `lib/wheel_frame` (the single-wheel WHL v1 formatter) is unchanged and separate from
  `lib/wheels_frame` (WHL2).
- The host-side parser in the FW repo was not checked from this repo. Agreement of the host parser
  with this format is not claimed here.

## 10. Verification status

Sim only. Rows (b) and (d) to (k) were captured by the owner with a host-side pyserial script over rfc2217://localhost:4002; the numbers were then checked by the main session against the saved capture files. Rows (a), (c) and (l) are owner-reported (row (l): observed in the Webots 3D view; the CSVs are owner-local and not in the repo). None of these is a hardware result.

| Run | Build and conditions | Result |
| --- | --- | --- |
| (a) VS Code Wokwi capture, 2026-11-05 | Build before #282 (single-ID reads) | 773 WHL2 lines parsed by the FW parser; no gaps; `rawL == -rawR`. |
| (b) VS Code route, pyserial capture, 2026-11-05 (scenario not identified; section 7 says it is not stated whether the scenario was used) | After #282 / B13, 25 s | 777 WHL2 lines parsed; 0 parse failures; 0 seq gaps; `rawL == -rawR`; state 0 only; 155 sim lines with numeric `pos1` and `pos2` and no `ERR`. Last `[dxl]` line: `tx_ok=832 tx_fail=0 rd_ok1=152 rd_fail1=0 rd_ok2=152 rd_fail2=0 max_us=5814`. `max_us` is on the simulated clock and is one longest transaction, not a real-bus number. The last 1 Hz `[dxl]` line (rd_ok1=152) precedes the final 3 of the 155 sim lines in that capture. Tags seen: `[ctrl_wheel]`, `[dxl]`, `[ttl_servo]`. The t_us span of the WHL2 lines is 16.496 s (777 lines, about 47 per sim second); the 25 s host capture window began before the sim produced lines, so no host-time ratio is given for this run (see the table in the note below). |
| (c) Webots plus bridge, 60 s | Bridge `--port rfc2217://localhost:4002`, owner run | Both wheels turned together; bridge exit code 0 (as the FW README states). Direction against sign: see row (l). |
| (d) preset 1 (counted run) | `-D C3_CTRL_WHEELS_TEST=1`, banner `[ttl_servo] TEST INJECT ON preset=1 after=100 (SIM-ONLY, remove before any other use)` seen in a capture from reset | 999 WHL2 lines (101 state 0, 898 state 2), 0 seq gaps. `[dxl] write failed id=1 raw=92 (n=1)` and `id=2 raw=92 (n=1)` once each; `[dxl] stop id=1 vel=0 ok=1` and `id=2 vel=0 ok=1`; `[safe] torque off before bus latch id=1` and `id=2` with no suffix; latch line `tx_ok=192 tx_fail=6 rd_ok1=20 rd_fail1=0 rd_ok2=20 rd_fail2=0 max_us=5814`; `[safe] bus down (reset the board to retry)`. WHL2 seq 99 and 100: state 0 with rawL 0 and rawR 0 (the two failing ticks, raws unknown); state 2 from seq 101 (the latch tick), 898 state-2 lines with rawL and rawR 0 and a nonzero changing model, seq contiguous across the latch. After the latch the 10 Hz line shows `cmd=NA pos1=NA dpos1=n/a pos2=NA dpos2=n/a gyroZ=.. bus=0` and only the three `[dxl] tx_ok` lines (two before, one latch) appear in the capture. |
| (e) preset 4 (counted run) | `-D C3_CTRL_WHEELS_TEST=4`, banner `[ttl_servo] TEST INJECT ON preset=4 after=100 (SIM-ONLY, remove before any other use)` seen from reset | 622 WHL2 lines, all state 0, 0 seq gaps. 10 sim lines with `pos2=ERR dpos2=n/a` while `pos1` stayed numeric; last 1 Hz line `rd_fail1=0 rd_fail2=10 tx_fail=0`; no `[safe]` line and no write-failed line. First good line after the window: `pos2=2274 dpos2=n/a`; the next: `pos2=2374 dpos2=100`. |
| (f) fall | `-D TTL_SERVO_FALL_TEST`, no injection (no banner expected) | 765 WHL2 lines (510 state 0, 255 state 1), 0 seq gaps. Boot line `[ttl_servo] FALL TEST: kp=2 kd=1 (expect FELL within ~6 s)`; 5 `[ttl_servo] FELL id=1 theta=...` lines (`\|theta\|` 0.785 to 0.832) and 5 `[ttl_servo] restart` lines; 10 stop lines, all `ok=1` (id=1 and id=2); no `[safe] torque off` line; no bus down. 5 state-1 holds of 51 WHL2 lines each, t_us span per hold 0.988, 0.988, 0.991, 0.991 and 1.002 s, model 0 and rawL and rawR 0 in all of them. |
| (g) preset 2 (NOT counted) | `-D C3_CTRL_WHEELS_TEST=2`; the capture started after boot (first captured WHL2 is seq 10), so the banner is not in the log | Not a counted preset run under the banner rule. Observations only: 1218 WHL2 lines (91 state 0, 1127 state 2), 0 seq gaps; one `[dxl] write failed id=2 raw=61 (n=1)` and no id=1 line; stop lines and `[safe] torque off before bus latch id=1` and `id=2`; latch line `tx_fail=3`; WHL2 seq 99 and 100 show rawL 61 and 68 (still updated) with rawR 0 at state 0; state 2 from seq 101 with raws 0; superseded by the counted re-run, row (i). |
| (h) preset 3 (NOT counted) | `-D C3_CTRL_WHEELS_TEST=3`; capture started after boot, no banner | Not a counted preset run. Observations only: 719 WHL2 lines, all state 0, 0 seq gaps; one `[dxl] write failed id=2 raw=-117 (n=1)`; no stop line, no `[safe]` line, no bus down; later 1 Hz lines show `tx_fail=2`; WHL2 seq 99 and 100 show rawL -117 with rawR 0, and rawR returns to 116 at seq 101; a second capture of the same preset also lacked the banner and was not kept, so it is not recorded here; superseded by the counted re-run, row (k). |
| (i) preset 2, counted re-run | `-D C3_CTRL_WHEELS_TEST=2`; capture from reset (starts at the ESP-ROM boot lines), banner `[ttl_servo] TEST INJECT ON preset=2 after=100 (SIM-ONLY, remove before any other use)` present | 992 WHL2 lines (101 state 0, 891 state 2), seq 0 to 991, 0 gaps. One `[dxl] write failed id=2 raw=23 (n=1)` and no id=1 line; `[dxl] stop id=1 vel=0 ok=1` and `id=2 vel=0 ok=1`; `[safe] torque off before bus latch id=1` and `id=2` with no suffix; latch line `tx_ok=201 tx_fail=3 rd_ok1=20 rd_fail1=0 rd_ok2=20 rd_fail2=0 max_us=5814`; `[safe] bus down (reset the board to retry)`. WHL2 seq 99 and 100: state 0, rawL 23 and 37 (the healthy wheel is still written), rawR 0; state 2 from seq 101 with rawL and rawR 0 for all 891 lines and a changing model. |
| (j) bring-up variant, servo2 deleted | default build (no injection flag, no banner in the log, expected); the owner deleted the `servo2` part and its four wires from `programs/c3_ctrl_wheels/diagram.json` in the working copy for the run only and restored the file afterwards (not committed); capture from reset | Boot lines in order: `[dxl] ping id=1 ok`, `torque_off id=1 ok=1`, `mode_velocity id=1 ok=1`, `goal0 id=1 ok=1`, `torque_on id=1 ok=1`, then `[dxl] ping id=2 FAILED`, `[dxl] bring-up id=2 failed`, `[dxl] bring-up failed, bus down, no Dynamixel calls` (printed before the forced stop), `[dxl] stop id=1 vel=0 ok=1`, `[dxl] stop id=2 vel=0 ok=0`, `[safe] torque off bring-up failed id=1`, `[safe] torque off bring-up failed id=2 (TORQUE OFF NOT CONFIRMED)`, latch line `[dxl] tx_ok=2 tx_fail=3 rd_ok1=0 rd_fail1=0 rd_ok2=0 rd_fail2=0 max_us=20019` (`max_us=20019` is the longest counted transaction since boot; which one it was is not shown; `tx_fail=3` fits the three tries of the zero write to ID 2 that the source makes, derived from the source and not shown in the log), `[safe] bus down (reset the board to retry)`. Then `[ttl_servo] gyro Z bias=...` and `Sim starts at 15 deg`; the 10 Hz lines show `cmd=NA pos1=NA ... pos2=NA ... bus=0`. 1052 WHL2 lines, seq 0 to 1051, 0 gaps, state 2 from the FIRST line (`WHL2,0,3002094,0.419200,0,0,2`), rawL and rawR 0 on all of them, the model changing. This is the first observation of the `(TORQUE OFF NOT CONFIRMED)` suffix; it was caused by the missing chip, not by an injected fault. |
| (k) preset 3, counted re-run | `-D C3_CTRL_WHEELS_TEST=3`; capture from reset (starts at the ESP-ROM boot lines), banner `[ttl_servo] TEST INJECT ON preset=3 after=100 (SIM-ONLY, remove before any other use)` present | 708 WHL2 lines, all state 0, seq 0 to 707, 0 gaps. One `[dxl] write failed id=2 raw=-39 (n=1)` and no id=1 line; no stop line, no `[safe]` line, no bus down (no latch); last 1 Hz line `tx_ok=1160 tx_fail=2 rd_ok1=138 rd_fail1=0 rd_ok2=138 rd_fail2=0 max_us=5758`. WHL2 seq 99 and 100: state 0, rawL -39 and -27, rawR 0 (the two failing ticks); seq 101: rawL -18, rawR 18 (recovered). |
| (l) Webots wheel direction, 2026-11-06 | Owner run of the FW bridge with Webots, default signs +1/+1, then `--sign-right -1`; observed in the Webots 3D view; CSVs owner-local, not committed | Positive command: both wheels turn about world +y in the same sense, rolling toward +x; sensor CSVs (owner-local, not in the repo, so this cannot be checked from the repo): `sign_pos.csv` (positive command run, both signs +1) shows both wheel sensors +1.000/+1.000 and an angle change of about +9.45 rad; `sign_neg.csv` (negative command run, both signs +1; not the `--sign-right -1` run) shows -1.000/-1.000 and about -9.47 rad, over about 9.5 s of sim time. `--sign-right -1` (commands +1/-1): erratic wheel velocities up to about 20 rad/s, above the 3.5 rad/s `maxVelocity` of the FW Webots world (owner-reported, not checked against the world file here); physics instability, cause not investigated, not a recommended setting. Chip-side signs not tested. |

Notes on rows (d) to (k): after each test run the owner rebuilt the default env and counted `TEST INJECT` strings in the ELF (0). The section 11.2 boot-log check (no banner line at boot after the rebuild) was done once, for the row (j) bring-up capture on the modified diagram (servo2 deleted; no banner in the log); it was not done for a run on the unmodified diagram after a rebuild. In this document, "non-injected failure" means a failure that is not injected; row (j) is a non-injected failure (a missing chip at bring-up). The counters (`tx_fail`, `rd_fail`) in rows (d), (e), (g), (h), (i) and (k) include injected failures (row (j) has no injection; its failures come from the missing chip). The injection skips the bus call, so these runs do not exercise the library's real timeout or error code.

Simulator speed against host time, for the eight captured runs. Ratio = t_us span of the WHL2 lines over the host-time span from the first to the last WHL2 line. Captured by the owner with a host-side pyserial script over rfc2217://localhost:4002; checked by the main session against the saved capture files. Sim only.

| Capture | WHL2 lines | Sim span (s) | Host span (s) | Sim / host | WHL2 per sim s |
| --- | --- | --- | --- | --- | --- |
| row (d) preset 1 | 999 | 20.181 | 21.425 | 0.94 | 49.5 |
| row (g) preset 2 (not counted) | 1218 | 24.454 | 24.833 | 0.985 | 49.8 |
| row (h) preset 3 (not counted) | 719 | 15.703 | 24.830 | 0.63 | 45.8 |
| row (e) preset 4 | 622 | 13.635 | 21.224 | 0.64 | 45.6 |
| row (f) fall | 765 | 16.359 | 22.024 | 0.74 | 46.8 |
| row (i) preset 2 (counted re-run) | 992 | 20.033 | 21.227 | 0.944 | 49.5 |
| row (k) preset 3 (counted re-run) | 708 | 15.474 | 20.620 | 0.75 | 45.8 |
| row (j) bring-up variant, servo2 deleted | 1052 | 21.022 | 21.027 | 1.00 | 50.0 |

The simulator ran between 0.63 and 1.00 of host time, so it can run slower than real time, and the speed differs from run to run. It is not known whether `--timeout` counts wall-clock or sim time.

Also:

- The single-wheel-era scratch runs described in `programs/c3_ctrl_wheels/README.md` predate the twin
  firmware. They are not evidence for this program.
- The native test `test/test_wheels_frame` was run by the main session on 2026-11-05: 11 of 11 test cases passed on main 8883d15 (`pio test -e test -f test_wheels_frame`), and the full `pio test -e test` passed 345 of 345 (20 suites) on main 394885c. Native tests check the formatter only and do not validate the firmware or hardware.

NOT RUN (a run that is recorded above as observed is not listed; presets 2 and 3 were each first captured without the banner (rows (g), (h), not counted) and re-run from reset (rows (i), (k), counted)):

- the preset 4 AFTER=0 variant
- 3-failed-tick latch from a non-injected failure of a chip that was working (row (j) is a bring-up failure with a missing chip, not the 3-tick path)
- bus-down state 2 after a non-injected failure of a chip that was working (state 2 after a non-injected bring-up failure is in row (j))
- the headless scenario run (`scenario_ctrl_wheels.yaml`)
- the planned wrong-wait-serial scenario run (#304)
- the section 11.2 boot-log check that the banner is gone after the rebuild, on the unmodified diagram (done once, for the row (j) capture on the modified diagram; the owner counted the ELF strings instead; see the note after the table)
- a failed zero write or torque-off from an injected fault or a non-injected mid-run failure (only the missing-chip case of row (j) was observed)
- a bring-up failure at ID 1
- a fall hold after a latch (source only)
- gyro-cal or no-IMU halt
- RFC2217 with three listeners
- a simulator restart mid-run and the firmware restart seq jump
- a long run and the seq/t_us wraps
- the chip-side wheel signs against Webots and against the servo (the net direction at the default bridge and Webots signs is row (l))
- latency
- real-bus read timing
- hardware

## 11. Failure-injection presets (V01, sim-only)

> **Status: SIM-ONLY, opt-in, not hardware-validated.** First owner-reported results (2026-11-05) are in
> section 10 rows (d) to (k). No V01 item outside rows (d) to (k) has been run.
> V01 is the failure-path verification set, tracked in #304 (results). Keep or remove is #309. This
> section records what `programs/c3_ctrl_wheels/main.cpp` says each preset does and which lines to look
> for. The lines below are expected from the source, not observed, except where section 10 records an
> observation. No hardware result.

### 11.1 What it is

- `-D C3_CTRL_WHEELS_TEST=1..4` is an opt-in, SIM-ONLY, UNVERIFIED failure injection for the twin-wheel
  fault paths. Default OFF. Set it only through `PLATFORMIO_BUILD_FLAGS`. Not hardware-validated.
- The windows below count `test_tick`: live, non-holding 20 ms passes, 1-based. A fall pass and a hold
  do not advance it.

| Preset | Injected failure | Window in `test_tick` (M = `C3_CTRL_WHEELS_TEST_AFTER`) |
| --- | --- | --- |
| 1 | Goal writes of both IDs fail | Ticks >= M, no end |
| 2 | Goal writes of ID 2 fail | Ticks >= M, no end (the latch at M+2 stops the writes; row (g)) |
| 3 | Goal writes of ID 2 fail, then recover | Ticks in [M, M+2): 2 ticks |
| 4 | 10 Hz position-read verdict for ID 2 fails (the real read still goes out) | Ticks in [M, M+50): about 1 s of sim time, about 10 reads at 10 Hz; holds do not advance the count, so the window stays active through a hold |

- `C3_CTRL_WHEELS_TEST_AFTER` (M) defaults to 100, about 2 s of live time at 50 Hz (the sim speed differs between runs;
  see the table in section 10). Without the TEST flag it has no effect, except that a negative value is an
  `#error` even then.
- AFTER=0 means every live pass from tick 1. Presets 1 and 2 fail on every such pass. Preset 3 fails only
  on the first pass (window [0, 2) over the 1-based counter). Preset 4 fails on passes 1 to 49 (derived
  from the window; the AFTER=0 variant was not run).
- Injected failures are counted by the real `[dxl]` bookkeeping, so the counters are NOT bus-only numbers.
- The full counting, latch and read rules are in the `C3_CTRL_WHEELS_TEST` comment block at the top of
  `main.cpp`; this section does not repeat them.

### 11.2 Build

```bash
PLATFORMIO_BUILD_FLAGS="-D C3_CTRL_WHEELS_TEST=1" pio run -e c3_ctrl_wheels-xiao_c3
```

- Use the one-shot prefix. Do not `export` it; an exported value stays in the shell and affects later
  builds.
- To set AFTER, add it to the same string, for example `-D C3_CTRL_WHEELS_TEST=2 -D C3_CTRL_WHEELS_TEST_AFTER=50`.
- A value outside 0..4 is an `#error`, and AFTER below 0 is an `#error`. `-D C3_CTRL_WHEELS_TEST` with no
  value defines it as 1 (preset 1). An empty value (`-D C3_CTRL_WHEELS_TEST=`) is a loud preprocessor
  error. A non-numeric token counts as 0 (off) and should build the normal firmware; this follows from the
  preprocessor rules and was not checked by a build.
- After testing, rebuild the default env with no flag. The VS Code run is expected to use the same build
  directory (from the owner's note; unverified). Check the firmware path in
  `programs/c3_ctrl_wheels/wokwi.toml`, because a stale test build is the leak risk.

```bash
pio run -e c3_ctrl_wheels-xiao_c3
```

- Confirm in the next boot log that there is no `[ttl_servo] TEST INJECT ON` line. Done once, for the row (j) capture on the modified diagram (no banner in cap_bring.log); not done for an unmodified-diagram run after a rebuild.

### 11.3 Banner rule

A result counts as a preset run only if the serial log, captured from reset, shows this line:

```
[ttl_servo] TEST INJECT ON preset=N after=M (SIM-ONLY, remove before uploading to a physical board)
```

The banner is printed once, in `setup()`, before the MPU6050 check. A capture started later can miss it.
A log without the banner is not counted as a preset run and is not proof of default firmware. Record the
banner line with each result. The banner text was changed on 2026-11-06 (owner decision, G10 option A): earlier captures, including rows (d), (e), (i) and (k) of section 10, show the old text "remove before any other use". The helper chk.py matches on the prefix "TEST INJECT ON preset=" and are not affected.

### 11.4 Lines to look for, per V01 item

The line formats below are copied from the format strings in `main.cpp`. The entries are source-derived expectations; items 1 and 2 (row (d)), item 3 (rows (i) for preset 2 and (k) for preset 3), and item 5 (row (e)) are observed with counted preset runs, and item 4 is observed in row (f) (not a preset run, no banner expected); everything else is expected.

#### Item 1: latch (preset 1)

Trigger: preset 1 at the default AFTER. Both goal writes fail from tick M. The third failed tick (M+2)
runs the forced stop and latches.

| Line to look for | Expected from the source |
| --- | --- |
| `[dxl] write failed id=1 raw=<raw> (n=1)` | Printed on the first failed tick only. Ticks M+1 and M+2 print nothing (n=2 and n=3 are not printed). |
| `[dxl] write failed id=2 raw=<raw> (n=1)` | Same tick, same n. `raw` is the value passed to `write_goal_raw`, before the wheel sign is applied (it can be negative). |
| `[dxl] stop id=1 vel=0 ok=1` and `[dxl] stop id=2 vel=0 ok=1` | Real zero writes, not injected. |
| `[safe] torque off before bus latch id=1` and `... id=2` | Printed with no suffix when the torque-off is confirmed; the suffix ` (TORQUE OFF NOT CONFIRMED)` is added only when it is not. |
| `[dxl] tx_ok=<n> tx_fail=<n> rd_ok1=<n> rd_fail1=<n> rd_ok2=<n> rd_fail2=<n> max_us=<n>` | The latch line. `tx_fail=6` (2 per failed tick, 3 ticks) if nothing else failed. |
| `[safe] bus down (reset the board to retry)` | Terminal. After it, the 10 Hz line shows `pos1=NA dpos1=n/a pos2=NA dpos2=n/a gyroZ=.. bus=0`, and no more 1 Hz `[dxl]` lines are printed. |

#### Item 2: state 2 (same run as item 1)

Trigger: the same run as item 1. State 2 is in the first WHL2 line written after the latch.

| Line to look for | Expected from the source |
| --- | --- |
| `WHL2,<seq>,<t_us>,<model>,0,0,2*HH` | `rawL` and `rawR` print 0 in state 2. `model` is not zeroed in state 2 (only while holding), so it can be nonzero. |
| `seq` | Advances by 1 per line. A gap means a dropped line (counted in `dropped` on the `[ctrl_wheel]` line). |

WHL2 lines keep coming after the latch, because the emit is outside the `servo_ok` branch, and the plants
keep running.

#### Item 3: one-ID write failure (presets 2 and 3)

Trigger: preset 2 (ID 2 writes fail from tick M on) or preset 3 (ID 2 writes fail on ticks M and M+1 only).

| Preset | Line to look for | Expected from the source |
| --- | --- | --- |
| 2 | `[dxl] write failed id=2 raw=<raw> (n=1)` only | No `id=1` write-failed line. Preset 2 still latches at M+2 (3-failed-tick rule), so the item 1 latch lines follow, with `tx_fail=3` on the latch line. |
| 3 | `[dxl] write failed id=2 raw=<raw> (n=1)`, once | Tick M+1 prints nothing (n=2 is not printed). |
| 3 | `[dxl] tx_ok=... tx_fail=...` (1 Hz line) | `tx_fail` rises by 2 across the window. |
| 3 | `[safe] bus down` | Not expected: there is no latch. |
| 3 | WHL2 `state` field | Stays 0. `rawR` prints 0 on the two failing ticks (unknown), then the confirmed value after recovery. |

#### Item 4: fall (existing `TTL_SERVO_FALL_TEST`, no injection)

Trigger: `PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_FALL_TEST" pio run -e c3_ctrl_wheels-xiao_c3`, with the
same one-shot and rebuild rules as section 11.2. Do not add the `C3_CTRL_WHEELS_TEST` flag.

| Line to look for | Expected from the source |
| --- | --- |
| `[ttl_servo] FALL TEST: kp=2 kd=1 (expect FELL within ~6 s)` | Boot line, printed after the gyro calibration. |
| `[ttl_servo] FELL id=1 theta=<n>` | `id=1` is expected: `W[0]` is checked first, and both plants get the same input and dt. `id=2` is not expected; it is reachable only if the plants diverged. |
| `[dxl] stop id=1 vel=0 ok=1` and `[dxl] stop id=2 vel=0 ok=1` | Non-forced stop. No `[safe] torque off` line is expected when both zero writes succeed (torque stays on at zero velocity). |
| WHL2 lines ending `,0,0,1*HH` | State 1 for about 1 s of `t_us`, with `model` 0.000000, `rawL` 0 and `rawR` 0. Judge the hold by `t_us`, not by line count; the simulator ran between 0.63 and 1.00 of host time across the eight captured runs (table in section 10), so judge the hold by `t_us`. |
| `[ttl_servo] restart` | After the 1 s hold. The state returns to 0. |

#### Item 5: read ERR (preset 4)

Trigger: preset 4 at the default AFTER. The ID 2 read verdict fails for ticks M to M+49, about 1 s of sim
time and about 10 reads.

| Line to look for | Expected from the source |
| --- | --- |
| 10 Hz line with `pos2=ERR dpos2=n/a` | During the window. `pos1` stays numeric. |
| 1 Hz `[dxl]` line | `rd_fail2` rises by about 10. `rd_fail1` stays 0 (observed in row (e)). |
| No `[safe]` lines and no `[dxl] write failed` lines | A read failure never latches and does not count as a write failure. |
| First good 10 Hz line after the window | `dpos2=n/a` (the failed read cleared the last position), then a numeric `dpos2` on the next line. |

#### Item 6: headless scenario

Not run. Section 7 describes the headless scenario `scenario_ctrl_wheels.yaml`. It has not been run, and
this section does not claim that the scenario drives any preset.

### 11.5 Not covered

- The library's real timeout and error code: the INJECTED write and read paths still do not exercise it. Row (j) shows non-injected library failures at bring-up: the ping to the missing ID 2 returned FAILED, the zero write to ID 2 returned ok=0, and the torque-off to ID 2 was not confirmed.
- A failed zero write, a failed torque-off, and the `(TORQUE OFF NOT CONFIRMED)` suffix: no preset makes
  a zero write or a torque-off fail, so the suffix is not reached (the missing-chip bring-up variant of row (j) did reach it, without injection). Presets 1 and 2 do run the stop path
  (through the 3-failed-tick latch) with real writes (see the header comment).
- Anything on hardware.

### 11.6 Capture guide (owner, VS Code Wokwi extension)

Steps (used by the owner for the section 10 rows (d) to (k), 2026-11-05):

1. The main session builds the preset ELF with the one-shot prefix (section 11.2).
2. Start the simulator for `programs/c3_ctrl_wheels`. The RFC2217 port is 4002 (`wokwi.toml`).
3. Start a small `python3 -I` pyserial script before (re)starting the sim, so the boot banner is captured.
   The script opens `rfc2217://localhost:4002`, retries until the port opens, prefixes each line with the
   host time, writes `cap_<item>.log`, and stops after about 25 s. pyserial is needed; not checked here.
   The capture script (`cap.py`) and the check script (`chk.py`) live outside this repo on the owner's machine and are not part of any PR.
4. Paste back `grep -v 'WHL2,' cap_<item>.log | head -80`, plus the WHL2 lines around the event.
5. Restore the default build afterwards (section 11.2): rebuild the default env without the flag and count the TEST INJECT strings in the ELF (`strings .pio/build/c3_ctrl_wheels-xiao_c3/firmware.elf | grep -c "TEST INJECT"`, expect 0; the owner reported 0). The boot-log check of section 11.2 (no banner line at boot) was done once, for the row (j) capture on the modified diagram; it was not done for an unmodified-diagram run after a rebuild.
   The ELF count is the check made for this step; the boot-log check is as stated above.

Bring-up variant (run by the owner on 2026-11-05, see section 10 row (j)): in the VS Code diagram editor, delete `servo2` and its wires.
Do not commit that change. Observed in row (j), in order:

- ID 1 comes up first: `[dxl] ping id=1 ok`, `[dxl] torque_off id=1 ok=1`, `[dxl] mode_velocity id=1 ok=1`,
  `[dxl] goal0 id=1 ok=1`, `[dxl] torque_on id=1 ok=1`.
- ID 2 fails: `[dxl] ping id=2 FAILED`, then `[dxl] bring-up id=2 failed`.
- `[dxl] bring-up failed, bus down, no Dynamixel calls` is printed BEFORE the forced stop, and the forced
  stop does make Dynamixel calls: `[dxl] stop id=1 vel=0 ok=1`, `[dxl] stop id=2 vel=0 ok=0`,
  `[safe] torque off bring-up failed id=1`, then `[safe] torque off bring-up failed id=2`, with
  ` (TORQUE OFF NOT CONFIRMED)` appended (observed for ID 2 in row (j)).
- Then the latch lines (`[dxl] tx_ok=...`, `[safe] bus down (reset the board to retry)`), and WHL2 state 2
  from the first WHL2 line.
