# c3_ctrl_wheel: WHL wheel-velocity relay (sim-only)

> **Status: SIM-ONLY, simulation-only, not hardware-validated.** The `WHL` relay route itself
> involves no MPU-6050, XL330 or STS3032 hardware (the Wokwi program still simulates them as in ttl_servo). Nothing here is a balance result, and the program makes no balance claim.
> The sign of `model` against Webots is UNVERIFIED. Latency is UNMEASURED.
>
> This is a RELAY, not a closed loop. Nothing is received from Webots. The firmware only prints.
>
> Dated 2026-11-02, from the scratch worktree runs described in section 8. Not run on the main
> branch by this document's author.

FW is freecad-workspace, the Python sim repo (Webots, bridge). BRC is this repo.

## 1. What it is

`programs/c3_ctrl_wheel/` is a sim-only Wokwi program for the XIAO ESP32-C3, env
`c3_ctrl_wheel-xiao_c3` (`platformio.ini`, `build_src_filter = -<*> +<c3_ctrl_wheel/>`).

- It is a copy of `programs/ttl_servo`. The Segway physics PD (`lib/segway_sim`), the Dynamixel
  XL330 velocity-mode writes over `Serial1` (GPIO20 RX, GPIO21 TX, 57600 baud, through the
  `wokwi-fe-urt-1` adapter), the fall hold and the bus latch are the same code as in `ttl_servo`; the fall and bus-down paths
  were not exercised in this copy.
- In addition, it prints one `WHL` line per 20 ms loop tick (`main.cpp`, the `whl_emit` call in
  `loop()`, after the 20 ms gate `LOOP_PERIOD_MS`). The line format is in section 3.
- It prints a 1 Hz `[ctrl_wheel] sent= dropped= max_emit_us=` counters line (section 4).
- The `WHL` lines are meant for the freecad-workspace bridge `poc/ctrl-wheel`. The program does not
  read anything from that bridge or from Webots.

## 2. Two values, never mixed

| Name | What it is | Where it comes from |
| --- | --- | --- |
| `model` | The firmware's lagged MODEL wheel velocity, rad/s. | `segway_sim.wheel_vel_rad_s` in `lib/segway_sim`. **Not a measurement.** It is the simulated plant's state. |
| `meas` | The Webots wheel sensor angle and its finite-difference velocity. | Computed on the freecad-workspace side from the Webots wheel sensor. **Not emitted by this firmware.** |

Rules for anyone reading the stream:

- Do not call `model` a measured wheel speed.
- Do not compare `model` and `meas` as if they were the same quantity without saying so. They come
  from different sources (the Wokwi sim model versus Webots).
- `raw` in the `WHL` line is derived from `model`, not from a measurement and not from the last
  value confirmed by the chip (section 3).

## 3. The `WHL` line

Format, one line per tick, terminated by `\n`:

```
WHL,<seq>,<t_us>,<model_rad_s>,<raw>,<state>*HH
```

Placeholders in the example are not real output. `HH` is the checksum described below.

| Field | Type and rule |
| --- | --- |
| `seq` | uint16, printed unsigned. Wraps 65535 to 0. Advances on every emit attempt, including a dropped line or a failed format, so a dropped line shows on the host as a sequence gap. |
| `t_us` | uint32, `micros()` at the moment the line is formatted. Wraps about every 71.6 minutes. It is the emit time, not a sample time. |
| `model_rad_s` | `%.6f`. A non-finite value is refused: no line is printed (counted in `dropped`, the `seq` gap shows). It is not reachable from the current firmware. A value that prints as `-0` drops its sign and prints as `0.000000`. |
| `raw` | Signed decimal, the chip raw goal velocity: `segway::wheel_cmd_to_raw(whl_model)`, capped at `segway::kWheelCapRaw` = 125 (magnitude, in raw units). Same raw units as the chip goal velocity. |
| `state` | One decimal digit. `0` run. `1` hold after a fall. `2` bus down. `bus_down` wins: if the bus is latched while holding, the state is `2`. |

State details:

- **Model forced to 0 while holding.** Whenever the firmware is holding after a fall, `model` is
  printed as 0 and `raw` is 0, whatever the state. A fall whose stop fails latches the bus and sets
  the hold in the same tick, so that tick prints state 2 (bus down wins) with model 0 and raw 0.
- **State 2 does not zero the model.** With the bus latched and no hold, `model` and `raw` still
  carry the live sim values. The chip is no longer written to, so these values are not what the
  chip is doing.
- **Non-finite model.** Refused by `wheel_frame::format` (returns 0), because a NaN or infinite value printed as `0.000000` would look like a genuine zero in run state. Not reachable today: a NaN or infinite plant state sets the fall flag in the same tick, so the hold line (state 1, model 0) is printed instead. The matching host change (freecad-workspace issue #475, PR #476, pending until merged) raises `WhlError` for the same input; the host's 200 ms silence rule is meant to stop the wheel then (host behaviour, not checked from this repo).

Checksum:

- `HH` is the XOR of every byte from `W` (the first byte of `WHL`) up to, but not including, `*`.
- It is written as two uppercase hex digits.
- The implementation is `lib/wheel_frame/wheel_frame.h`, which reuses `lib/imu_frame` for the
  checksum and the `%.6f` and `-0` rules. There is one copy of each.

Emission:

- The line is written only if the whole line fits in `Serial.availableForWrite()`. The buffer is
  `whl_buf[96]`. A line is about 45 bytes (the source comment; not measured).
- Otherwise the line is dropped and counted as `dropped`. Whether `Serial.write` can still block on the C3 USB CDC is NOT
  verified, and the loop's other prints (10 Hz line, `[dxl]`, FELL) are unguarded, so a stalled host
  can still block the loop; the extra TX load of about 2 kB/s (estimate) makes that more likely.

Sync with the host side:

- The reference format is freecad-workspace `poc/ctrl-wheel/ctrl_wheel_msg.py`. Any format change
  must be made in both repos. This document did not check the Python file.

## 4. Counters line

```
[ctrl_wheel] sent=<n> dropped=<n> max_emit_us=<n>
```

- Printed once per second, and only when `Serial.availableForWrite() >= 80`. The 80-byte gate is a
  sizing choice for one line, not measured. If the gate fails, the line is skipped and printed at
  the first later tick where the gate passes.
- `sent` and `dropped` are **cumulative since boot**. They are never reset. At 50 Hz with no drops,
  `sent` grows by about 50 per second, so the first line shows about 50.
- `dropped` also counts a refused frame (a non-finite model, section 3). It does not distinguish a refused frame from a full USB buffer.
- `max_emit_us` is the longest `whl_emit` call since the previous counters line. It resets on each
  print. The timing is the simulated clock, not a C3 measurement.

## 5. Build and run

```bash
make c3_ctrl_wheel                 # build the env (c3_ctrl_wheel-xiao_c3), NOT run from this document
make wokwi-chips                   # build the shared chips (wokwi-fe-urt-1, wokwi-ttl-servo) that wokwi.toml loads; needs network on first use
make sim-c3_ctrl_wheel-xiao_c3 WOKWI_ARGS="--scenario scenario_ctrl_wheel.yaml --timeout 20000"   # needs WOKWI_CLI_TOKEN
```

- The sim target runs headless `wokwi-cli`. The CLI never exits on its own, so **exit 42 from the
  timeout is expected** for the run above.
- The env has its own build directory (`.pio/build/c3_ctrl_wheel-xiao_c3`, from `wokwi.toml`), so it
  does not share a build directory with `ttl_servo`.
- Fall path (state 1): build with `PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_FALL_TEST"` (low gains
  kp=2, kd=1, so the frame falls). Not exercised in Wokwi from this copy (see section 6).
- The `[ctrl_wheel]` and `WHL` lines go to the same USB CDC serial stream as the `[ttl_servo]`
  and `[dxl]` log lines. The log tags in this copy still say `[ttl_servo]` and `[dxl]`.

RFC2217:

- `programs/c3_ctrl_wheel/wokwi.toml` sets `rfc2217ServerPort = 4001`. Port 4000 is `ttl_servo`'s
  port, so the two can run at the same time in principle.
- A simulator run that honors the key exposes the simulated serial port as a plain TCP server, with
  no authentication. Its bind address was not checked. The same risk is described for port 4000 in
  `docs/TTL_SERVO_DOF_FRAMES.md` (section "VS Code simulator route (RFC2217)").
- The VS Code extension keeps its listener on the port open after the simulation stops (observed
  for port 4000 only, see CLAUDE.md; NOT checked for 4001). Check with
  `ss -ltn` before a bridge-first run.
- RFC2217 read on port 4001: NOT run.

## 6. What is NOT claimed

- **No closed loop through Webots.** This is a relay. The firmware sends `WHL` lines out and reads
  nothing back from Webots or the bridge.
- **No balance.** Not a controller result, not a balance validation, and not a gain check.
- **No hardware.** Not run on a C3 board, and nothing here is hardware-validated.
- **Sign UNVERIFIED.** The sign of `model` against the Webots wheel is not checked.
- **Latency UNMEASURED.** The 80-byte gate, the 96-byte buffer and the `max_emit_us` figures are
  sizing choices or simulated-clock values, not measurements.
- **State 1 (hold after a fall) and state 2 (bus down) are not yet exercised in Wokwi.** The
  headless run showed state 0 only.
- **The non-finite refusal is not exercised in Wokwi.** The condition cannot occur in the current
  firmware, so the decision is covered by the native `test_wheel_frame` test only.
- **`meas` is not produced here.** Only `model` is printed.
- The Wokwi physics constants are guesses (`lib/segway_sim`), so the `model` values do not describe
  a real robot.

## 7. Origins

- Copied from `programs/ttl_servo`:
  - `main.cpp`: copied, with the IMU frame (TTL_SERVO_IMU_FRAMES), DOF, EXT_CMD, KILL_POC and HORN_SELFTEST blocks stripped (the MPU6050 gyro Z read stays) and
    the FALL_TEST block kept. The `WHL` line and the counters line were added.
  - `wokwi.toml`: copied, with `rfc2217ServerPort = 4001` (ttl_servo uses 4000).
  - `diagram.json`: copied, minus the `kill_btn` pushbutton.
  - `scenario.yaml`: copied as `scenario_ctrl_wheel.yaml`.
- Reused, unchanged: `lib/segway_sim` (model and PD), `lib/dxl_units` (raw conversion), `lib/imu_frame`
  (through `lib/wheel_frame`), and the shared chips in `wokwi/chips/` (`wokwi-fe-urt-1`,
  `wokwi-ttl-servo`).
- `programs/c3_ctrl_wheel/README.md` was refreshed in the same PR to match this text.

## 8. Verification status

Only the following were run, all in a scratch worktree, all sim-only. The two headless runs used the
working trees that became commit 13c049f (counters, run 2) and its parent on branch feat/245 (run 1);
the later review-round edits only removed dead lines (`dof_rest`, `math.h`) and changed comments, and
the build was re-run after them (succeeded), the Wokwi runs were not repeated. One later change is
behavioural: issue #261 makes `wheel_frame::format` refuse a non-finite model. The headless Wokwi runs
predate it and do not cover it (the condition cannot occur in the simulation); only the native test does. The parser was
`ctrl_wheel_msg.parse_line` from freecad-workspace PR #462 as of commit 05bc267 (before its review fixes):

| Command or check | Result |
| --- | --- |
| native `test_wheel_frame` | 19/19 passed in the earlier runs; 20/20 after the non-finite change (issue #261); the whole native suite passed 334/334 on commit d9cc19c (run on 2026-11-02). The commits after d9cc19c change docs and comments only |
| `pio run -e c3_ctrl_wheel-xiao_c3` | succeeded on commit d9cc19c; `ttl_servo-xiao_c3` and `balance-xiao_c3` also succeeded on d9cc19c |
| headless `wokwi-cli` run, 12 to 15 s, stdout, exit 42 from the timeout | two runs showed 608 and 458 `WHL` lines; all lines were accepted by the freecad-workspace parser, with no `seq` gaps and only state 0 |
| counters line during the second headless run (12 s, commit 13c049f; the first run predates the counters code) | `sent=50 dropped=0` at the first 1 s line (the counts are cumulative, so later lines grow by about 50 per second) |

Not run:

- RFC2217 read on port 4001.
- Fall path (state 1) and bus-down path (state 2) in Wokwi.
- Webots, with this stream or any closed loop.
- Hardware of any kind.
- The `WHL` format against the freecad-workspace parser beyond what is listed above.
- Latency and timing on a real C3.
