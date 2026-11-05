# c3_ctrl_wheels: twin-wheel WHL2 relay (sim-only)

> **Status: SIM-ONLY, simulation-only, not hardware-validated.** This is a RELAY, not a closed loop:
> the firmware prints WHL2 lines and receives nothing back from Webots. No balance claim. The wheel
> signs are UNVERIFIED. Latency is UNMEASURED. Dated 2026-11-05.

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
  from the source; not run.
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
- There are three independent sign places, and none is confirmed: (1) the chip signs above;
  (2) the bridge `--sign-left` and `--sign-right` flags, default +1 and +1, applied to the model before
  the clamp (the raw columns never reach the command); (3) the Webots controller constants
  `SIGN_LEFT = 1` and `SIGN_RIGHT = 1`. The flags are described in the
  [FW README, Sign convention](https://github.com/pluto-atom-4/freecad-workspace/blob/main/poc/ctrl-wheels/README.md#sign-convention).
- `rawL == -rawR` is expected by construction. Both plants get identical inputs, and the wire value is
  the sign times the same raw. It can differ only after one wheel's write or torque-off has left its
  last-confirmed value out of step with the other's. It is not evidence of two independently behaving
  wheels.

## 5. Stop and latch rules

Each rule is marked with its simulator status. "Not run in a simulator" means no run has shown the
path; the owner captures in section 10 only cover the normal running path.

- **Bring-up (`setup()`).** For ID 1 then ID 2: ping, torque off, velocity mode, goal 0, torque on.
  It stops at the first failure, so ID 2 is not tried if ID 1 fails. On any failure the firmware sends
  a forced stop to both IDs (zero velocity, then torque off) and latches. The success path is implied
  by the numeric `pos1` and `pos2` in the section 10 (b) run. The failure path is not run in a
  simulator.
- **Per tick (`loop()`).** Both goal writes are always attempted. A tick fails if either write fails.
  A write on change (the signed value equals `last_raw`) makes no bus call and counts as a success.
  The write path is seen in section 10 (state 0). The failure path is not run in a simulator.
- **Three failed ticks in a row.** A forced torque-off on both IDs (zero velocity first, up to 3
  tries per step), then the latch `bus_down`. The latch is terminal: no Dynamixel calls, reset the
  board to retry. Not run in a simulator.
- **Fall (either plant `has_fallen`).** A zero-velocity stop on both IDs. Torque off is sent only if a
  zero write failed. If the stop returns false (any zero write failed), the bus latches. Then a 1 s
  hold: the plants are frozen and no writes go out. Then both plants restart and the failure streak
  resets. Not run in a simulator (the owner-reported capture (b) in section 10 shows state 0 only).
- **After a latch.** No goal writes, and no stop is sent on a fall. The hold and restart still run,
  and the state stays 2.
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
  `[dxl] bring-up id=.. failed`; and once for the whole bring-up, `[dxl] bring-up failed, bus down, no Dynamixel calls`.
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
  is a judgement chosen to leave room, not derived or measured, and the sim ran slower than real time (about 31
  WHL2 lines per second, computed from the 777 WHL2 lines in the 25 s capture of section 10, row b). Not run. The sim is headless
  only and is not a balance test. A failed or missing wait ends at `--timeout` (exit 42), so exit code 42
  does not prove the pings matched. A deliberately wrong wait-serial string run (recording the real exit code of a failed wait) is planned in V01 (#304), not run.
- The fall path needs `PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_FALL_TEST"` (low gains kp=2, kd=1). Not run.
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
  - the wheel sign against Webots or against the servo;
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

Owner-reported, sim only. None of these is a hardware result.

| Run | Build and conditions | Result |
| --- | --- | --- |
| (a) VS Code Wokwi capture, 2026-11-05 | Build before #282 (single-ID reads) | 773 WHL2 lines parsed by the FW parser; no gaps; `rawL == -rawR`. |
| (b) VS Code Wokwi capture, 2026-11-05 (scenario not identified; section 7 says it is not stated whether the scenario was used) | After #282 / B13, 25 s | 777 WHL2 lines parsed; 0 parse failures; 0 seq gaps; `rawL == -rawR`; state 0 only; 155 sim lines with numeric `pos1` and `pos2` and no `ERR`. Last `[dxl]` line: `tx_ok=832 tx_fail=0 rd_ok1=152 rd_fail1=0 rd_ok2=152 rd_fail2=0 max_us=5814`. `max_us` is on the simulated clock and is one longest transaction, not a real-bus number. Tags seen: `[ctrl_wheel]`, `[dxl]`, `[ttl_servo]`. The sim ran slower than real time (about 31 WHL2 lines per second). |
| (c) Webots plus bridge, 60 s | Bridge `--port rfc2217://localhost:4002`, owner run | Both wheels turned together; bridge exit code 0 (as the FW README states). Direction against sign is not settled. |

Also:

- The single-wheel-era scratch runs described in `programs/c3_ctrl_wheels/README.md` predate the twin
  firmware. They are not evidence for this program.
- The native test `test/test_wheels_frame` was not run by the author of this document.

NOT RUN:

- 3-failed-tick latch
- bus-down state 2
- one-ID write failure
- the headless scenario run (`scenario_ctrl_wheels.yaml`)
- a fall seen in WHL2 (state 1)
- the ERR read path
- a bring-up failure at ID 2
- gyro-cal or no-IMU halt
- RFC2217 with three listeners
- a simulator restart mid-run and the firmware restart seq jump
- a long run and the seq/t_us wraps
- Webots wheel direction vs sign
- latency
- real-bus read timing
- hardware
