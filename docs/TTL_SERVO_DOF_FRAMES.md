# ttl_servo DOF frame stream (opt-in)

> **Status: SIM-ONLY opt-in diagnostic, NOT hardware-validated.** Nothing on this page is a
> balance result. The frames are synthesised from the Segway sim state, not measured. The sign
> map is derived and UNVERIFIED (#185). No hardware (MPU-6050, XL330, STS3032) is involved.
>
> Written against the branch stack for epic #184: PR #194 (S1, `lib/segway_imu`), PR #195,
> and PR #196 (S4, capture script), stacked on `feat/189-dof-capture-script` (03db189). None of
> these PRs is merged yet. Until they merge, every command below refers to that stack.
>
> Every command below is marked **RUN** (executed by the main session on 2026-10-22, with the
> result stated) or **NOT run**.

## Overview

With `-D TTL_SERVO_DOF_FRAMES`, the `ttl_servo` firmware emits `IMU,<seq>,<t_us>,...` frames
built from the SegwaySim state (`theta`, `theta_dot`). They are not read from the Wokwi MPU or
the servo chip. A capture script turns a Wokwi serial log into the raw CSV that the FW bridge
`--replay` reads, so the bridge and Webots can be fed a sim-derived stream.

Pieces, all in this repo:

- `lib/segway_imu/segway_imu.h`: pure synthesis, `from_state()` (S1, #186).
- `programs/ttl_servo/main.cpp`: the DOF block, behind the flag (S2, #187).
- `programs/ttl_servo/scenario_dof_frames.yaml`: headless Wokwi scenario (S3, #188).
- `tools/ttl_servo_dof_capture.sh`: Wokwi log to raw CSV (S4, #189). Run `-h` for its options.

The frame layout and checksum come from `lib/imu_frame` and are the same as the
`tilt_servo` stream. See `docs/TILT_SERVO_FRAMES.md` for that stream. It is a different
stream, for a different program, and its results do not carry over to this one.

## Flag and build commands

The DOF stream is compiled when `-D TTL_SERVO_DOF_FRAMES` is defined. The default is OFF.
`PLATFORMIO_BUILD_FLAGS` is the route used on this page.

```bash
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" pio run -e ttl_servo-xiao_c3   # RUN: OK, RAM 4.5%, flash 23.4% (main session, 2026-10-22)
pio run -e ttl_servo-xiao_c3                                                     # RUN: OK, default build (main session, 2026-10-22)
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES -D TTL_SERVO_IMU_FRAMES" pio run -e ttl_servo-xiao_c3   # RUN: FAILS with the intended #error (programs/ttl_servo/main.cpp:35)
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_IMU_FRAMES" pio run -e ttl_servo-xiao_c3    # RUN: OK (POC build only, main session, 2026-10-22)
pio test -e test                                                                 # RUN: 272/272, 17 suites (S1 branch, main session, 2026-10-22)
```

- The build directory `.pio` is shared. After any flagged build, run the default build line
  again. The main session did this after every flagged build.
- No upload was run. `-t upload` is NOT run on any line here.

## What is sent

Each frame is built from the sim state after the physics step. Sign table from #185 section 2.
It is DERIVED from the rotation convention and UNVERIFIED. It was not checked against a
datasheet, Webots or Wokwi.

Convention: nose = +X, `theta > 0` = lean forward, `theta = 0` upright.

| field | value | note |
| --- | --- | --- |
| ax | `-sin(theta)` g | |
| ay | 0 | |
| az | `+cos(theta)` g | +1 g when flat |
| gx | 0 | |
| gy | `theta_dot` x 57.29578 deg/s | pitch rate, clamped to +-250 dps |
| gz | 0 | yaw. The chip gyro Z is never copied into the frame |

Check by hand, NOT run: `theta = +15 deg` gives `ax = -0.259`, `az = 0.966`, and
`atan2(0.259, 0.966) = +15 deg` under the host formula `atan2(-ax, sqrt(ay^2 + az^2))`.

- Gravity rotation only. There is no wheel-acceleration term.
- Deterministic: no noise, no bias, no tare, no filter. The bridge owns bias and tare.
- `seq` starts at 0 and advances on every attempted frame. `t_us` is the emit time, not a
  sample time.
- The frame is not sent on a fall tick, while the body is holding after a fall, or after the
  stream is latched off (see "Phases").

## Phases

1. **Rest.** Physics is frozen at `theta = 0` with the wheel at 0, for `kDofRestTicks = 150`
   loop ticks. That is about 3 s at the nominal 50 Hz. The count is in ticks, not ms. Frames
   flow during rest with `ax = 0`, `az = 1`, `gy = 0`.
2. **PD sim and stimulus.** After rest, `SegwaySim` runs its PD from `theta = 0`. The stimulus
   is the Wokwi MPU gyro Z (`rotationZ`, deg/s). After boot bias averaging it feeds the sim as a
   disturbance. It only moves the body. It never reaches the frame `gz`.
3. **Fall, stream latched off.** On a fall the firmware prints `[ttl_servo] FELL ...`, then
   `[frames] stopped (fall)` once. No DOF frames are sent again until the board is reset. The
   chip stop path is unchanged: zero velocity, up to 3 tries, torque off if all fail. After the
   1 s hold, the chip restart resets SegwaySim to the 15 deg start angle
   (`SIM_INIT_THETA_RAD = 0.262 rad`). No frames resume.
   NOT run: no run exercised the fall path with the DOF flag on. The scenario has no fall. This
   is from reading `programs/ttl_servo/main.cpp` only.

## Capture script

The script runs `wokwi-cli` headless on `programs/ttl_servo` and converts the serial log to the
raw bridge CSV (header `seq,t_us,ax,ay,az,gx,gy,gz`). Its `-h` text is the authority for the
options. It never builds firmware, and it does not verify checksums.

```bash
make wokwi-chips                                  # RUN, prerequisite for the capture run (main session, 2026-10-22; needs network only if the chip .wasm files are missing)
WOKWI_CLI_TOKEN=<token> tools/ttl_servo_dof_capture.sh -o /abs/path/ttl_servo_dof_raw.csv   # RUN: exit 0, 399 rows (default scenario and timeout)
tools/ttl_servo_dof_capture.sh -c /abs/path/existing.log -o /abs/path/out.csv              # NOT run (convert-only mode)
shellcheck tools/ttl_servo_dof_capture.sh                                                    # NOT run
```

- The capture run above used a DOF build (see the flag section). The script never rebuilds.
- Result of the capture run (RUN): 399 rows, 0 dropped, `seq` 0..398.
- Independent check by the main session (RUN; done outside the script, method not recorded
  here): 399 of 399 frame checksums valid; `gx` and `gz` exactly 0 in every row; `gy` span 0
  over the first 50 rows (the FW arm gate needs a span of 5 dps or less over 50 frames);
  `max |gy|` 7.1 deg/s; `max |ax|` 0.071 g.
- `t_us` steps ran from 14.7 to 26 ms. That is simulator timing, NOT a measured rate.

## Wokwi headless run

```bash
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" make sim-ttl_servo-xiao_c3 WOKWI_ARGS="--scenario scenario_dof_frames.yaml --timeout 20000"   # RUN: exit 0, "Scenario completed successfully"
make sim-ttl_servo-xiao_c3 WOKWI_ARGS="--scenario scenario_dof_frames.yaml --timeout 20000"                                                   # RUN, negative control on the default build: wokwi-cli timeout, make exit 2 / "Error 42"
```

- The DOF run gave 399 `IMU,` lines and a 1 Hz counter line `[frames] sent=150 dropped=0`,
  `max_emit_us` about 1.7 ms. Both are on the SIMULATED clock, and `max_emit_us` is not a
  real-C3 value. No fall. A 30 deg/s gyro Z push gave a sim `theta` of about 0.04 rad.
- The scenario checks only the banner, the `IMU,` lines and the counter line. It does not check
  checksums, signs or motion. `WOKWI_CLI_TOKEN` must be set for both lines.
- NOT run: the Wokwi GUI gyro Z push by hand (S0 item 4, deferred).

## FW side: Webots replay

The FW code is in freecad-workspace (FW), not in this repo. Start Webots first, then the
bridge. The commands are from the FW README sections "Wheel bridge (dof_wheel_bridge, #375)"
and the Webots `run_demo.sh`. The bridge path in the FW repo is `poc/esp32-dof/monitor/`, so
run the bridge from the FW repo root, not from `poc/esp32-dof`.

```bash
cd <freecad-workspace>/poc/servo-wheel-dof && SWD_UDP_PORT=5006 ./run_demo.sh --gui   # NOT run (Webots start)
cd <freecad-workspace> && mamba run -n esp32-dof python3 poc/esp32-dof/monitor/dof_wheel_bridge.py --replay /abs/path/ttl_servo_dof_raw.csv --core lqr --udp-port 5006   # NOT run
```

- The `--replay` CSV needs the header above and at least 50 frames. The capture run's CSV
  meets both. The bridge's own arm (50 frames, span 5 dps or less) was NOT run on this CSV.
- The bridge sends the wheel command to Webots only (UDP port 5006). It does not write to the
  ttl_servo board or the chip.
- `PITCH_SIGN` and `WHEEL_DISPLAY_SIGN` are both 1.0 in the FW host code, UNVERIFIED (FW #377).
- Owner-reported (2026-10-22, #185 comments, not independently checked, conditions not
  recorded): in Webots, a nose-down +20 deg replay leaned the model forward. The LQR wheel
  turned toward the lean and the PID wheel turned away. This is the FW #359 PID vs LQR sign
  conflict, still open.
- NOT run: the bridge `--replay` on this CSV, and Webots with this CSV.

## What this is NOT

- **Open loop.** The sim does not read the bridge. Nothing from Webots or the bridge feeds back
  into the ttl_servo physics or the chip.
- **Not a balance result.** The sim physics constants are guesses. Nothing here validates a
  controller, a gain or a sign.
- **Gravity-rotation only.** There is no linear-acceleration term, so accel reads the same
  whether the body is still or moving at that angle.
- **Deterministic.** No noise, bias, filter, DLPF lag or sensor jitter. `t_us` follows
  simulator timing.
- **Yaw is zero.** `gz` is always 0. The bridge `yaw` output is also always 0.0.
- **Display wheel is an integral.** It is a commanded velocity integrated over time, not a
  measured wheel. Levelling the body stops it without rewinding it.
- **Not hardware.** Nothing on the MPU-6050, the XL330, the STS3032 or the real C3 is shown.

## Known UNVERIFIED items

- The sign map in "What is sent" (derived from #185 section 2). Not checked against a datasheet,
  Webots or Wokwi.
- `PITCH_SIGN` and `WHEEL_DISPLAY_SIGN` (FW #377), and the PID vs LQR sign conflict (FW #359).
- Direction of the Wokwi gyro Z push on the GUI (not checked by hand).
- The fall path with the DOF flag on (code read only, not run).
- The bridge's behaviour on this CSV: arming, wheel direction and Webots motion (not run).
- Checksums: the capture script does not verify them by design. The main session's check is
  the only one, and its method is not recorded here.
- `shellcheck` on `tools/ttl_servo_dof_capture.sh` (NOT run).
- Everything on hardware.

## Two frame modes, mutually exclusive

`TTL_SERVO_IMU_FRAMES` is the earlier raw-gyro POC (#160). It is still in the tree, and its
removal is tracked in #166 (OPEN). `TTL_SERVO_DOF_FRAMES` is this stream. Defining both fails
the build with `#error` at `programs/ttl_servo/main.cpp:35` (RUN, see the flag section). Use
one flag at a time. The POC's results are not results for this stream.

## Cross-links

- FW #10 (Webots parent), FW #377 (human GUI check; sign constants), FW #359 (PID vs LQR sign
  conflict).
- BRC #160 (ttl_servo POC), #166 (POC removal, OPEN), #184 (epic), #185 (S0 sign table),
  #191 (S6 human replay, OPEN).
- `docs/TILT_SERVO_FRAMES.md` (the `tilt_servo` stream, a different program).
