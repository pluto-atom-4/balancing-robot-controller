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
wheel. Without `-c`, nothing feeds back into the physics or the chip, so there is no closed loop;
with `-c` the command is sent back to the firmware (see "Closed loop (Stage 2)" below).

The script never builds firmware. It checks `WOKWI_CLI_TOKEN`, the two chip `.wasm` files, the
bridge file under `FW_ROOT` and `.pio/build/ttl_servo-xiao_c3/firmware.bin`, then runs the pipeline.
The bridge uses `python3` from PATH; the FW env is `mamba run -n esp32-dof`, and the whole script
can be run under it.

```bash
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" pio run -e ttl_servo-xiao_c3   # build with the flag (built OK on 2026-10-29)
make wokwi-chips                                                                   # NOT run: only if the chip .wasm files are missing
cd <FW>/poc/servo-wheel-dof && SWD_UDP_PORT=5006 ./run_demo.sh --gui              # NOT run: in its own terminal, keep open (world servo_wheel_dof, UDP 5006)
FW_ROOT=<FW> WOKWI_CLI_TOKEN=<token> tools/ttl_servo_dof_live.sh                   # run headless without Webots on 2026-10-29; the Webots display NOT run; from the BRC repo root
tools/ttl_servo_dof_live.sh -h                                                     # prints usage, exits 0 (nothing else runs)
pio run -e ttl_servo-xiao_c3                                                       # NOT run: afterwards rebuild without the flag (shared .pio)
```

Options: `-s SCENARIO`, `-t TIMEOUT_MS` (default 60000), `-f FW_ROOT`, `-c` (closed loop, Stage 2 below), `-h`. Anything after `--` goes to
the bridge unchanged, for example `-- --core pid`; a later `--core` wins over the built-in
`--core lqr`. The bridge sends UDP to 127.0.0.1:5006 by default, the port the Webots world listens on.
Exit codes are in the script's `-h` text (the authority).

- Run length: about 15 s of wall time for the default scenario (about 8 s until the first frame, then about
  7 s of scenario steps); wokwi-cli then exits 0 and the bridge sees end of input. `-t` only matters if the
  scenario does not finish, and exit 0 does not prove it finished (a short `-t` gives wokwi-cli exit 42,
  one stderr line, and exit 0 when the bridge exited 0). A build without the flag never prints the
  banner the scenario waits for, so wokwi-cli exits 42 after `-t` and the bridge gets no frames; the bridge
  then exits 2 ("never armed") and the script returns that code. For a longer demo use `-s scenario_dof_live.yaml` (gyro Z pushes of +30 and -30 deg/s, about 20 s of scenario steps after the first frame; give the bridge `--duration 45` or more).
- Observed once on 2026-10-29 with `-s scenario_dof_live.yaml -- --duration 45 --record` (headless, no Webots):
  exit 0, 1587 rows recorded, `cmd_rad_s` went from +1.0 to -1.0 and `wheel_angle` ended near 0. The status line
  showed `faults=1` from the first push on; the cause was not investigated, so treat the swing as an
  observation, not as validated controller tracking.
- Observed on 2026-10-29 (headless, no Webots): with `-t 6000` wokwi-cli printed "Timeout: simulation did not finish
  in 6000ms" and exited 42, the bridge had armed and exited 0 (168 rows recorded), and the script exited 0 with the
  one stderr line about exit 42. With a build WITHOUT the flag and `-t 15000` the same wokwi-cli timeout message
  appeared, the bridge printed "never armed: window incomplete: 0/50" (exit 2) and the script exited 2.
- Run Webots in its own terminal and keep it open; the script is started from a second terminal.
- Ctrl-C is meant to stop wokwi-cli and the bridge together (one foreground process group); this was not run. A SIGTERM or SIGHUP sent only to the script is not forwarded.
- Observed on 2026-10-29 (headless, no Webots, T2 bridge branch): wokwi-cli needs about 8 s before the first
  frame, and the bridge polls meanwhile, so its `ticks` count runs ahead of the frames; use a bridge
  `--duration` of at least 15 s. If the bridge exits first (one run with `--duration 3`, where the bridge
  itself failed with "duration expired before the arm window filled"), wokwi-cli crashes with an unhandled
  Node `EPIPE` error and a stack trace. The script returns the bridge's exit code first; if the bridge had
  exited 0, it would return wokwi-cli's nonzero code instead. A bridge exit 1 and the script's own exit 1
  look the same.

| Part | Status |
| --- | --- |
| DOF frames from the SegwaySim state, built with the flag | Run headless in Wokwi once (see "Wokwi headless run" and "Capture script") |
| wokwi-cli stdout carries serial lines live through a pipe | Spike on 2026-10-29, not the script (a Python harness reading the pipe): one headless 9 s run, 308 `IMU,` lines spread over the run; checksums and rate not checked |
| `tools/ttl_servo_dof_live.sh` | Run headless 7 times on 2026-10-29 (no Webots): (1) default scenario, died at the exit-code handling (`PIPESTATUS` bug, fixed) after the bridge had armed and recorded 557 rows; (2) default scenario with `-- --duration 3`: the bridge error plus the wokwi-cli `EPIPE` crash; (3) default scenario: exit 0, 565 rows; (4) `-s scenario_dof_live.yaml -- --duration 45`: exit 0, 1587 rows; (5) `-t 12000`: exit 0, 561 rows, wokwi-cli finished before the limit (no exit 42 line); (6) `-t 6000`: wokwi-cli exit 42, bridge exit 0, 168 rows, script exit 0; (7) build without the flag, `-t 15000`: bridge exit 2, script exit 2 |
| Bridge `--stdin` source (freecad-workspace) | Script runs 3 to 5 above went through it: armed, `cmd_rad_s` rose to +1.0 (the bridge's limit) after the gyro Z push, `wheel_angle` integrated to about 5.8 rad (run 3). Also, not the script: a captured Wokwi log piped on stdin gave exit 0 and 259 rows. The status line showed `faults=1` in runs 1, 4 and 5; cause not investigated |
| Webots display of the LQR wheel | Not run |
| Sign conventions | UNVERIFIED |
| Closed loop | None in the default mode (no `-c`); `-c` is Stage 2 below, NOT run |

NOT run: the Webots display with this stream (the live script and bridge were run headless only, without Webots); sign conventions UNVERIFIED (pluto-atom-4/balancing-robot-controller#185,
pluto-atom-4/freecad-workspace#359, pluto-atom-4/freecad-workspace#377); no closed loop in the default mode.

### Closed loop (Stage 2)

Opt-in with `-c`. SIM-ONLY, not hardware-validated, and NOT run end to end (see the NOT-run line below).
The script creates a private temp dir with a FIFO (`cmd.fifo`), runs wokwi-cli with `--interactive`
reading that FIFO, and passes `--cmd-out` to the bridge so it writes `CMD <rad_s>` lines into the FIFO
(the bridge option is pluto-atom-4/freecad-workspace#435). The temp dir and FIFO are removed on exit.

```bash
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES -D TTL_SERVO_EXT_CMD" pio run -e ttl_servo-xiao_c3   # NOT run
pio run -e ttl_servo-xiao_c3                                                                         # NOT run: afterwards rebuild without the flags (shared .pio)

Run length in closed-loop mode (observed 2026-10-29): `wokwi-cli --interactive` does not exit after a scenario has
completed; the simulation stops and it waits for its standard input to close, which never happens with `-c`. So use
`scenario_dof_closed.yaml` (the two gyro pushes, then a very long delay, so it never completes) together with an explicit
`-t` such as 45000: the run ends at `-t` with wokwi-cli exit 42, which counts as normal. The first closed-loop attempt with
`scenario_dof_live.yaml` completed its scenario at 32 s of simulated time and then hung until the outer timeout killed it.
FW_ROOT=<FW> WOKWI_CLI_TOKEN=<token> tools/ttl_servo_dof_live.sh -c -s scenario_dof_closed.yaml -t 45000 -- --cmd-scale -1   # run headless once; runs end to end but does not balance, see below
```

- The firmware flag `-D TTL_SERVO_EXT_CMD` is pluto-atom-4/balancing-robot-controller#221. The script
  cannot check it. Without the flag the commands are ignored and the run is open loop.
- With the flag, the firmware PD is replaced by the host command while that command is fresher than
  200 ms. The `[ext_cmd] rx= bad= used= pd=` line counts them. The field meanings are as named here;
  the firmware side is not in this checkout, so they were not checked against the code.
- The stream stops when the sim body falls, as in the default mode.
- The sign of the Python LQR command versus the firmware sim is UNVERIFIED. If the loop diverges, use
  `-- --cmd-scale -1` (flips the bridge command sign). Sign UNVERIFIED: pluto-atom-4/freecad-workspace#359,
  pluto-atom-4/balancing-robot-controller#185.
- A scenario whose first steps use `wait-serial` works together with `--interactive` (verified by the
  main session in a scratch project on 2026-10-29: a scenario plus `--interactive` from a FIFO delivered
  the typed line to the firmware and the scenario finished). A `wait-serial` for text printed BEFORE that
  step starts would not match, so a closed-loop scenario should not wait for text that appears early.

| Part | Status |
| --- | --- |
| FIFO, `wokwi-cli --interactive < FIFO`, bridge `--cmd-out` wiring in the script | NOT run |
| Firmware with `TTL_SERVO_EXT_CMD` replacing the PD while the command is fresh | Flag only tested by typing constant CMD lines (the body fell as expected for a constant command); NOT run through the script |
| `[ext_cmd] rx= bad= used= pd=` counters | NOT run |
| Temp dir and FIFO removal on exit | NOT run |
| Early exit of one side (bridge or wokwi-cli) while the other holds the FIFO | NOT checked |
| Sign of the command (`-- --cmd-scale -1`) | UNVERIFIED |
| Webots display in closed loop | NOT run |
| Hardware | None; nothing on hardware |

Observed on 2026-10-29 (headless, no Webots; the firmware flag branch, the bridge `--cmd-out` branch and this launcher mode merged
into a throwaway local branch): the mechanics work. `-c -s scenario_dof_closed.yaml -t 45000` ended by itself after 53 s with exit 0
(wokwi-cli exit 42 at `-t`), removed its FIFO directory, and the firmware reported `[ext_cmd] rx=531 bad=0 used=907 pd=24`, so the
host commands reached and drove the sim. It does NOT balance: with `--cmd-scale 1` and with `--cmd-scale -1`, with the +30/-30 deg/s
pushes and also with a small 8 deg/s push of 1 s, the body fell about 1.3 to 1.5 s after the host command took over (8 to 20 falls per
run, always to theta about +0.8) and the frame stream stops at the first fall. Neither sign recovered, so the sign question
(pluto-atom-4/freecad-workspace#359) is not settled by this. Unverified hypotheses: the bridge command is limited to +-1.0 rad/s
(the recorded `cmd_rad_s` range was 0 to +1.0) while the firmware PD can use up to +-3 rad/s; the loop delay through
`wokwi-cli --interactive` (about 10 to 12 accepted `CMD` lines per second were counted, far below the bridge's write rate); the LQR gains
come from another plant model than the firmware sim. Not run: Webots with this mode; anything on hardware.

## VS Code simulator route (RFC2217)

> **SIM-ONLY, NOT hardware-validated. Webots: wheel movement observed by the owner with this route (2026-10-30), not measured; see the Webots paragraph below.** Dated 2026-10-30.

What it is: `programs/ttl_servo/wokwi.toml` sets `rfc2217ServerPort = 4000`, merged via
pluto-atom-4/balancing-robot-controller#227. When a simulator front end that honors the key is
running (SELF-REPORTED, main session, 2026-10-30: the owner's VS Code Wokwi extension run and a
headless `wokwi-cli` v0.27.1 run both opened the port), the bridge read the simulated serial port
over RFC2217 with `--port rfc2217://localhost:4000` on 2026-10-30 (self-reported, see below)
instead of the stdin pipe of `tools/ttl_servo_dof_live.sh`. That script is not used on this route.

Steps:

```bash
cd <this repo>
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" pio run -e ttl_servo-xiao_c3
```

Then, in VS Code, open the Wokwi simulator for `programs/ttl_servo` (its `wokwi.toml` and
`diagram.json`) and start it. Keep the simulator tab visible, because the Wokwi docs say a hidden
tab may pause the simulation. Keep the gyro Z slider at 0 during boot, because boot averages gyro
Z for about 1 s. Start the simulator FIRST, then run the bridge:

```bash
cd <FW>/poc/esp32-dof/monitor
mamba run -a "" -n esp32-dof python3 dof_wheel_bridge.py --port rfc2217://localhost:4000 --core lqr --duration 15 --record out.csv --echo-text
```

The `-a ""` in the command above makes mamba's output live; without it mamba holds the output until the end. This is the command used in the self-reported 2026-10-30 run (main session), run from this directory; the Stage 1 text above runs the bridge from the repo root with the full poc/esp32-dof/monitor/ path, which is also valid. Afterwards restore the default build, because the
`.pio` directory is shared:

```bash
pio run -e ttl_servo-xiao_c3
```

Result: on 2026-10-30 this command, run by the main session against the owner's VS Code simulation
(DOF build), armed with `faults=0`, recorded 969 rows and exited 0. SELF-REPORTED by the main
session, not independently checked. Results of the acceptance run go to discussion 216 of
pluto-atom-4/balancing-robot-controller. The plan is discussion 228 of the same repo. The
acceptance issue is pluto-atom-4/balancing-robot-controller#226.

Webots (owner observation, 2026-10-30): the owner reported that the Webots wheel moved forward and backward while the owner moved the gyro Z slider back and forth, with the VS Code simulation read through `--port rfc2217://localhost:4000`. This was not recorded or measured by the main session. Signs are UNVERIFIED, so it only shows the pipe works.

Start order and retry: start the simulator before the bridge. A bridge started with a URL `--port` before the port is open prints `[dof_serial] waiting for rfc2217://localhost:4000 (attempt 1): ...` and retries every 1 s, repeating the line when the error text changes and at least every 10 s. The wait ends when the port opens, at the `--duration` deadline (the bridge then exits 2 with "duration expired before the arm window filled"), on Ctrl+C (exit 2 if not yet armed), or after 300 s (DofSerialError "did not open within 300 s", exit 2). A device path (no "://") still fails at once with exit 2. A typo URL, such as a wrong port number, also waits up to the cap. The first-open retry is MERGED: pluto-atom-4/freecad-workspace#440 (for pluto-atom-4/freecad-workspace#439). SELF-REPORTED by the main session, 2026-10-30, headless stand-in simulation: the bridge started first printed the wait line (attempt 1, and again at attempt 11), then armed with `faults=0` when the simulation came up. NOT reported for the owner's VS Code run: the wait line, the armed line and the exit code.

Listener left open by VS Code: SELF-REPORTED (main session, 2026-10-30): after the owner stopped the simulation in VS Code, the VS Code process still listened on port 4000 (seen with `ss -ltn`), and a pyserial client could open it in 0.4 s and read 0 bytes. In that state a bridge-first run shows no wait line and just sees no data. Close the simulator tab or reload the VS Code window until `ss -ltn | grep 4000` shows nothing, before a bridge-first run. Cause not checked.

Fallback: the `wokwi-stream.sh` relay from pluto-atom-4/freecad-workspace#438 (it lives in the
freecad-workspace repo at `poc/servo-wheel-dof/tools/wokwi-stream.sh`) piped into the bridge
`--stdin`. The pipe is NOT verified. An ad-hoc relay of about 10 lines piped into `--stdin` did
arm the bridge on 2026-10-30 (self-reported).

Risk note. SELF-REPORTED (main session, 2026-10-30): while the simulation ran, `ss -ltn` showed
`*:4000` (a wildcard bind; the bind address was not checked further), so the port may be reachable
on all interfaces. The port is plain TCP without authentication. If the port is reachable, anyone who can reach the machine could read the simulated serial output
(the read side rests on the self-reported run above); a write reaching the simulation was NOT
tested, and the default build has no serial command reader, so a write has an effect only with the
`-D TTL_SERVO_EXT_CMD` build (UNVERIFIED). Impact is low: the data is simulated, there is no hardware
and no secrets. The owner accepted this risk (discussion 228, decision 4). Do not run the simulator on an
untrusted network. SELF-REPORTED (main session, 2026-10-30): after the simulator was stopped, a
listener on 4000 was still seen once (cause not checked).

NOT run or UNVERIFIED:

- Restarting the simulation mid-run.
- A long run (pacing against wall clock).
- A headless run and a GUI run at the same time. The toml opens port 4000 on every run of that
  project, so they may clash.
- The bind address.
- The fallback pipe.
- Signs (UNVERIFIED, pluto-atom-4/balancing-robot-controller#185 and
  pluto-atom-4/freecad-workspace#359).

## What this is NOT

- **Open loop.** In the default mode nothing from Webots or the bridge feeds back into the ttl_servo
  physics or the chip. The bridge wheel command goes to Webots only. With `-c` see Stage 2 above.
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
- RFC2217 route: self-reported runs on 2026-10-30, Webots wheel movement observed by the owner, restart and long run not run; details in the section "VS Code simulator route (RFC2217)".

## Cross-links

- FW #10 (Webots parent), FW #359 (PID vs LQR sign conflict), FW #377 (human GUI check; sign
  constants).
- BRC #160 (ttl_servo POC), #166 (POC removal, OPEN), #184 (epic), #185 (sign table),
  #186 (`lib/segway_imu`), #187 (DOF block), #188 (scenario), #189 (capture script),
  #190 (this doc), #191 (human replay through the bridge, OPEN), #192 (OPEN, owner: CLAUDE.md pointer).
- PRs #194 (`lib/segway_imu` and the DOF block), #195 (scenario), #196 (capture script), #197 (this doc).
- `docs/TILT_SERVO_FRAMES.md`.
- RFC2217 route: pluto-atom-4/balancing-robot-controller#226 (closed), pluto-atom-4/balancing-robot-controller#227,
  pluto-atom-4/balancing-robot-controller#229, discussion 228 and discussion 216 of
  pluto-atom-4/balancing-robot-controller, pluto-atom-4/freecad-workspace#438,
  pluto-atom-4/freecad-workspace#439, pluto-atom-4/freecad-workspace#440, pluto-atom-4/balancing-robot-controller#231.
