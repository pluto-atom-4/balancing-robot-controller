# ttl_servo DOF frame stream (opt-in)

> **Status: SIM-ONLY opt-in diagnostic. NOT hardware-validated, and no hardware is involved**
> (no MPU-6050, XL330 or STS3032). Nothing here is a balance result. The frames are
> synthesised from the Segway sim state, not measured. The sign map is derived and UNVERIFIED
> (#185).
>
> As of 2026-10-22: PRs #194 (S1+S2), #195 (S3) and #196 (S4) are merged into main (merge
> commits 173c2aa, a13e14e, 5ea6bbc); this page is PR #197 (S5). #166, #191 and #192 are OPEN.
> The S1 test run (272/272) was at commit 9503d49. The commands marked RUN were run before the
> merges, on the branch commits that #194-#196 then merged.
>
> The main session executed the commands marked RUN on 2026-10-22. A line marked "builder run
> (self-reported)" was run by the builder agent, not the main session. Each command line is
> marked RUN (main session), builder run (self-reported), or NOT run.

FW is freecad-workspace, the Python sim repo (Webots, bridge). BRC is this repo.

## Overview

With `-D TTL_SERVO_DOF_FRAMES`, the `ttl_servo` firmware emits `IMU,<seq>,<t_us>,...` frames
built from the SegwaySim state (`theta`, `theta_dot`). They are not read from the Wokwi MPU or
the servo chip. A capture script turns a Wokwi serial log into the raw CSV that the FW bridge
`--replay` reads, so the bridge and Webots can be fed a sim-derived stream.

Pieces in BRC:

- `lib/segway_imu/segway_imu.h`: pure synthesis, `from_state()` (#186).
- `programs/ttl_servo/main.cpp`: the DOF block behind the flag (#187).
- `programs/ttl_servo/scenario_dof_frames.yaml`: headless Wokwi scenario (#188).
- `tools/ttl_servo_dof_capture.sh`: Wokwi log to raw CSV (#189). Run `-h` for its options.

The frame layout and checksum come from `lib/imu_frame`, the same as the `tilt_servo` stream.
That stream is a different program; see `docs/TILT_SERVO_FRAMES.md`. Its results do not carry
over to this one.

## Flag and build commands

The DOF stream is compiled when `-D TTL_SERVO_DOF_FRAMES` is defined. The default is OFF.
`PLATFORMIO_BUILD_FLAGS` is the route used on this page.

```bash
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" pio run -e ttl_servo-xiao_c3   # RUN: OK, RAM 4.5%, flash 23.4%
pio run -e ttl_servo-xiao_c3                                                     # RUN: OK (default build)
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES -D TTL_SERVO_IMU_FRAMES" pio run -e ttl_servo-xiao_c3   # RUN: FAILS with the intended #error (programs/ttl_servo/main.cpp:35)
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_IMU_FRAMES" pio run -e ttl_servo-xiao_c3    # builder run (self-reported): OK
pio test -e test                                                                 # RUN: 272/272 at commit 9503d49 (17 suites); includes test/test_segway_imu
```

- The two frame flags are mutually exclusive. The combined build fails with the `#error` above.
  `TTL_SERVO_IMU_FRAMES` is the earlier raw-gyro POC (#160). It is still in the tree, and its
  removal is tracked in #166 (OPEN). POC results do not apply to this stream.
- The `.pio` build directory is shared. Run the default build line again after any flagged
  build; the main session did this after every flagged build.
- No upload was run.

## What is sent

Each frame is built from the sim state after the physics step. The sign table is from #185
section 2. It is DERIVED from the rotation convention, not checked against a datasheet, Webots
or Wokwi.

Convention: nose = +X, `theta > 0` = lean forward, `theta = 0` upright.

| field | value | note |
| --- | --- | --- |
| ax | `-sin(theta)` g | |
| ay | 0 | |
| az | `+cos(theta)` g | +1 g when flat |
| gx | 0 | |
| gy | `theta_dot` x 57.29578 deg/s | pitch rate, clamped to +-250 dps |
| gz | 0 | yaw; the chip gyro Z is never copied into the frame |

- The +15 deg and -15 deg rows are covered by the native tests `test_table_row_plus_15deg` and
  `test_table_row_minus_15deg` in `test/test_segway_imu` (RUN, `pio test -e test`, 272/272).
  The host formula step, `atan2(-ax, sqrt(ay^2 + az^2))` giving +15 deg for those values, was
  checked by hand only.
- Gravity rotation only: there is no wheel-acceleration term. Deterministic: no noise, bias,
  tare or filter. The bridge owns bias and tare.
- `seq` starts at 0 and advances on every attempted frame. `t_us` is the emit time, not a
  sample time.
- No frame is sent on a fall tick, while holding after a fall, or once the stream is latched
  off (see "Phases").

## Phases

1. **Rest.** Physics is frozen at `theta = 0` with the wheel at 0, for `kDofRestTicks = 150`
   loop ticks: about 3 s at the nominal 50 Hz. The count is in ticks, not ms. Frames flow
   during rest with `ax = 0`, `az = 1`, `gy = 0`.
2. **PD sim and stimulus.** After rest, `SegwaySim` runs its PD from `theta = 0`. The stimulus
   is the Wokwi MPU gyro Z (`rotationZ`, deg/s), averaged at boot and then fed to the sim as a
   disturbance. It moves the body only; it never reaches the frame `gz`.
3. **Fall, stream latched off.** NOT run (code read only; read from `programs/ttl_servo/main.cpp`).
   On a fall the firmware prints `[ttl_servo] FELL ...`, then
   `[frames] stopped (fall)` once. The DOF stream stays off until the board is reset. The chip
   stop path is unchanged: zero velocity, up to 3 tries, torque off if all fail. After the 1 s
   hold, the chip restart resets SegwaySim to the 15 deg start angle
   (`SIM_INIT_THETA_RAD = 0.262 rad`), and no frames resume.
   The scenario has no fall, so the fall path with the DOF flag on was not exercised.

## Capture script

The script runs `wokwi-cli` headless on `programs/ttl_servo` and converts the serial log to the
raw bridge CSV (header `seq,t_us,ax,ay,az,gx,gy,gz`). Its `-h` text is the authority for the
options. It never builds firmware. It reads whatever is in the build directory, so the DOF
build must be made first (the script's `-h` "Build first" step). A stale or default build gives
the wrong log. Afterwards restore the default build, because `.pio` is shared.

```bash
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" pio run -e ttl_servo-xiao_c3   # RUN (see Flag and build commands)
make wokwi-chips                                                                   # RUN (prerequisite of the capture run)
WOKWI_CLI_TOKEN=<token> tools/ttl_servo_dof_capture.sh -o /abs/path/ttl_servo_dof_raw.csv   # RUN: exit 0, 399 rows (default scenario and timeout)
tools/ttl_servo_dof_capture.sh -c /abs/path/existing.log -o /abs/path/out.csv      # NOT run (convert-only mode)
shellcheck tools/ttl_servo_dof_capture.sh                                          # NOT run
```

- Result: 399 rows, 0 dropped, `seq` 0..398.
- The script does not verify checksums, by design. The main session checked them separately
  (RUN): 399 of 399 valid. It also found `gx` and `gz` exactly 0 in every row, `gy` span 0
  over the first 50 rows, `max |gy|` 7.1 deg/s, `max |ax|` 0.071 g. `t_us` steps ran from
  14.7 to 26 ms; that is simulator timing, not a rate.

## Wokwi headless run

```bash
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" make sim-ttl_servo-xiao_c3 WOKWI_ARGS="--scenario scenario_dof_frames.yaml --timeout 20000"   # RUN: exit 0, "Scenario completed successfully"
make sim-ttl_servo-xiao_c3 WOKWI_ARGS="--scenario scenario_dof_frames.yaml --timeout 20000"   # RUN, negative control on the default build: wokwi-cli timeout, make exit 2 / "Error 42"
```

- The DOF run gave 399 `IMU,` lines. The `[frames]` counter line is CUMULATIVE, printed once
  per second. It read `sent=50,100,150,200,250,300,350` with `dropped=0` on every line.
  `max_emit_us` was between 1649 and 2298. All of these are on the SIMULATED clock, not
  real-C3 values, and nothing here is measured on a C3.
- No fall. A 30 deg/s gyro Z push gave a maximum sampled sim theta of 0.071 rad (about 4 deg),
  from the 79 `[ttl_servo] sim theta=` lines at 10 Hz. That agrees with `max |ax|` 0.071 g in
  the capture.
- The scenario checks only the banner, the `IMU,` lines and the counter line. It does not check
  checksums, signs or motion.
- NOT run: the Wokwi GUI gyro Z push by hand (#185 item 4).

## FW side: Webots replay

Per the FW README and FW code (not checked in BRC): the bridge arms after 50 frames of
stillness (the "arm gate", span 5 dps or less over those frames); it sends UDP only and never
writes to the board; its `yaw` output is always 0.0; `PITCH_SIGN` and `WHEEL_DISPLAY_SIGN` are
both 1.0, UNVERIFIED (FW #377).

Start Webots first, then the bridge. `<FW>` is the freecad-workspace repo root. The bridge runs
from the repo root because `poc/esp32-dof/monitor/...` does not resolve from `poc/esp32-dof`.

```bash
cd <FW>/poc/servo-wheel-dof && SWD_UDP_PORT=5006 ./run_demo.sh --gui                       # NOT run
cd <FW> && mamba run -n esp32-dof python3 poc/esp32-dof/monitor/dof_wheel_bridge.py --replay /abs/path/ttl_servo_dof_raw.csv --core lqr --udp-port 5006   # NOT run
```

- The capture CSV has the header and more than 50 frames. Per the FW README / FW code, not checked
  in BRC, `--replay` needs both. The bridge arm was NOT run on this CSV.
- Owner-reported (2026-10-22, #185 comments; not independently checked; conditions not
  recorded): in Webots, a nose-down +20 deg replay leaned the model forward. The LQR wheel
  turned toward the lean and the PID wheel turned away. This result is NOT tied to this capture
  CSV or this build (the owner's replay conditions were not recorded); a replay of this stream
  is #191 (OPEN).

## Live stream (Stage 1)

`tools/ttl_servo_dof_live.sh` runs the same headless Wokwi scenario as the capture script, but
pipes wokwi-cli's stdout live into the FW bridge `--stdin` instead of writing a CSV. It is SIM-ONLY,
a one-way stream, and not hardware-validated. Two controllers are involved: the firmware PD drives
the Wokwi servo chip (the physics), and the Python LQR command is only shown on the Webots display
wheel. Nothing feeds back into the physics or the chip, so there is no closed loop.

The script never builds firmware. It checks `WOKWI_CLI_TOKEN`, the two chip `.wasm` files, the
bridge file under `FW_ROOT` and `.pio/build/ttl_servo-xiao_c3/firmware.bin`, then runs the pipeline.
The bridge uses `python3` from PATH; the FW env is `mamba run -n esp32-dof`, and the whole script
can be run under it.

```bash
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" pio run -e ttl_servo-xiao_c3   # build with the flag (built OK on 2026-10-29)
make wokwi-chips                                                                   # NOT run: only if the chip .wasm files are missing
cd <FW>/poc/servo-wheel-dof && SWD_UDP_PORT=5006 ./run_demo.sh --gui              # NOT run: in its own terminal, keep open (world servo_wheel_dof, UDP 5006)
FW_ROOT=<FW> WOKWI_CLI_TOKEN=<token> tools/ttl_servo_dof_live.sh                   # run headless without Webots on 2026-10-29; the Webots display NOT run; from the BRC repo root
tools/ttl_servo_dof_live.sh -h                                                     # run, exits 0
pio run -e ttl_servo-xiao_c3                                                       # NOT run: afterwards rebuild without the flag (shared .pio)
```

Options: `-s SCENARIO`, `-t TIMEOUT_MS` (default 60000), `-f FW_ROOT`, `-h`. Anything after `--` goes to
the bridge unchanged, for example `-- --core pid`; a later `--core` wins over the built-in
`--core lqr`. The bridge sends UDP to 127.0.0.1:5006 by default, the port the Webots world listens on.
Exit codes are in the script's `-h` text (the authority).

- Run length: the default scenario ends after about 9 s; wokwi-cli then exits 0 and the bridge sees end of
  input. `-t` only matters if the scenario does not finish. A build without the flag never prints the
  banner the scenario waits for, so wokwi-cli exits 42 after `-t` and the bridge gets no frames; the bridge
  then exits 2 ("never armed") and the script returns that code. Use `-s` with a longer scenario for a longer demo.
- Run Webots in its own terminal and keep it open; the script is started from a second terminal.
- Ctrl-C stops wokwi-cli and the bridge together. A SIGTERM or SIGHUP sent only to the script is not forwarded.
- Observed on 2026-10-29 (headless, no Webots, T2 bridge branch): wokwi-cli needs about 8 s before the first
  frame, and the bridge polls meanwhile, so its `ticks` count runs ahead of the frames; use a bridge
  `--duration` of at least 15 s. If the bridge exits first (one run with `--duration 3`), wokwi-cli crashes
  with an unhandled Node `EPIPE` error and a stack trace; the script then returns the bridge's exit code.

| Part | Status |
| --- | --- |
| DOF frames from the SegwaySim state, built with the flag | Run headless in Wokwi once (see "Wokwi headless run" and "Capture script") |
| wokwi-cli stdout carries serial lines live through a pipe | Spike on 2026-10-29, one headless 9 s run: 308 `IMU,` lines spread over the run; checksums and rate not checked |
| `tools/ttl_servo_dof_live.sh` | Run headless 3 times on 2026-10-29 (no Webots); the last run exited 0 after the script's `PIPESTATUS` bug was fixed |
| Bridge `--stdin` source (freecad-workspace) | Run once through the live pipe: armed, `cmd_rad_s` rose to +1.0 (the bridge's limit) after the gyro Z push, `wheel_angle` integrated to about 5.8 rad, 565 rows recorded (`--record`); one earlier run showed `faults=1` near the end, cause not investigated |
| Webots display of the LQR wheel | Not run |
| Sign conventions | UNVERIFIED |
| Closed loop | None, by design |

NOT run: the Webots display with this stream (the live script and bridge were run headless only, without Webots); sign conventions UNVERIFIED (pluto-atom-4/balancing-robot-controller#185,
pluto-atom-4/freecad-workspace#359, pluto-atom-4/freecad-workspace#377); no closed loop.

## What this is NOT

- **Open loop.** Nothing from Webots or the bridge feeds back into the ttl_servo physics or the
  chip. The bridge wheel command goes to Webots only.
- **Display wheel is an integral.** It is a commanded velocity integrated over time, not a
  measured wheel. Levelling the body stops it without rewinding it.
- **PID vs LQR sign conflict** (FW #359) is open. The owner-reported result above is one
  observation, not a fix.
- The sim physics constants are guesses, so nothing here validates a controller or a gain.
- Not hardware (see the banner).

## Known UNVERIFIED items

- Sign map: see "What is sent".
- `PITCH_SIGN`, `WHEEL_DISPLAY_SIGN` and the wheel direction on Webots: see "FW side" (FW #377,
  FW #359).
- GUI gyro-Z push direction: see "Wokwi headless run".

## Cross-links

- FW #10 (Webots parent), FW #359 (PID vs LQR sign conflict), FW #377 (human GUI check; sign
  constants).
- BRC #160 (ttl_servo POC), #166 (POC removal, OPEN), #184 (epic), #185 (sign table),
  #186 (`lib/segway_imu`), #187 (DOF block), #188 (scenario), #189 (capture script),
  #190 (this doc), #191 (human replay through the bridge, OPEN), #192 (OPEN, owner: CLAUDE.md pointer).
- PRs #194 (`lib/segway_imu` and the DOF block), #195 (scenario), #196 (capture script), #197 (this doc).
- `docs/TILT_SERVO_FRAMES.md`.
