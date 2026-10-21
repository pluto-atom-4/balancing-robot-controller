# tilt_servo IMU frame stream (opt-in)

> **Status: opt-in diagnostic stream, NOT hardware-validated.** The `tilt_servo` frame stream
> has NOT been run in any simulator or on hardware. The sim and replay results on this page
> are for the ttl_servo POC stream only, never the tilt_servo stream.
>
> Written against `balancing-robot-controller` `main` 8e9d698 and the freecad-workspace (FW)
> merge `ab3c4f4` (PR #382). The FW docs may change: FW #376 is open.

Nothing on this page is a balance result. Every command below is marked **RUN** (executed for
this page, with the result stated) or **NOT run**.

## Overview and cross-links

This page repeats only the minimum a tilt_servo user needs. The FW README is the source:
`poc/esp32-dof/README.md` in freecad-workspace, merge `ab3c4f4` (PR #382). The overview bullets
below name the FW README sections. The "Host side" and "Capture and replay" sections name theirs.

- Frame shape: section "Frame protocol".
- Arming, bias/tare ownership, `yaw` always 0.0, text-line handling: section "Wheel bridge
  (dof_wheel_bridge, #375)", subsection "Behaviour".
- Replay CSV header: section "Wheel bridge (dof_wheel_bridge, #375)", subsection "Two CSV formats".
- Monitor: section "Monitor".

The FW-side facts on this page (arm window, bias and tare ownership, text-line handling, `yaw`
0.0, the `dof_frame.format_frame` and `SerialImuHal` names, and the `esp32_dof.ino` settings)
were read from the merged FW files at `ab3c4f4`. They were NOT re-run, except for the offline
replay in the "Capture and replay" section.

Related tracking:

- FW #376: docs, open.
- FW #377: human GUI check, open.
- FW #387: optional bridge option to echo or log text lines, open.
- Local format source: `lib/imu_frame/imu_frame.h`.
- In this repo: #157 (the stream, merged), #172 (issue, merged via PR #173: free-space
  gate), #159 (human bench run), #160 (ttl_servo POC tracker).

Frame line shape (full specification in the FW README):

```
IMU,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>*HH
```

## Flag and build commands

The stream is compiled when `-D TILT_SERVO_IMU_FRAMES` is defined. `PLATFORMIO_BUILD_FLAGS`
is the route used on this page. Any route that defines the macro compiles the stream. The
default is OFF.

```bash
PLATFORMIO_BUILD_FLAGS="-D TILT_SERVO_IMU_FRAMES" pio run -e tilt_servo-xiao_c3   # RUN: success, no warnings or errors (main session, 8e9d698)
pio run -e tilt_servo-xiao_c3                                                     # RUN: success (default build, main session, 8e9d698)
PLATFORMIO_BUILD_FLAGS="-D TILT_SERVO_IMU_FRAMES" pio run -e tilt_servo-xiao_c3 -t upload   # NOT run
pio device monitor -e tilt_servo-xiao_c3                                          # NOT run
```

Build flags come from the environment of each command. Changing them rebuilds the firmware.
This was observed for builds on 8e9d698. The upload with the flag was NOT run. So the flag must
be set on the upload command too. Without it, the upload uses whatever build the environment
selects. Expected inference, NOT run: no frames appear in that case.

Bridge commands: the live bridge run (`--port`) is described in the FW README section "Wheel
bridge". NOT run for `tilt_servo`. Only the replay commands in "Capture and replay" are given
here.

## Cost (measured on 8e9d698)

Build sizes from the main session (RUN, builds only):

| Build | RAM | Flash |
| --- | --- | --- |
| default | 14680 B | 312092 B |
| `-D TILT_SERVO_IMU_FRAMES` | 14832 B | 313476 B |
| difference | +152 B | +1384 B |

Size numbers say nothing about timing or behaviour. They do not show the stream's effect on
the loop.

## What is sent

Checked against `programs/tilt_servo/main.cpp`:

- The values are RAW. Accel is sent in g and gyro in deg/s, converted from the Adafruit
  library units before formatting.
- Ranges: accel +-4 g, gyro +-500 dps. DLPF is 21 Hz.
- No boot-time bias removal and no filter.
- The firmware sign `GYRO_PITCH_SIGN` is NOT applied to the raw gyro y value in the frame.
- `t_us` is the loop-start `micros()` value, not the IMU sample time.
- Compared with the POC firmware `esp32_dof.ino`, the ranges match (accel +-4 g, gyro
  +-500 dps). The differences are the bias handling (the POC runs about 1 s of boot gyro bias
  calibration; tilt_servo runs none), the DLPF (POC 44 Hz, tilt_servo 21 Hz) and the
  sample-rate setting (the POC sets divisor 9, 100 Hz; tilt_servo sets no divisor). The effect
  of these differences is UNMEASURED.
- The C++ formatter maps NaN and inf to 0.0. The Python `dof_frame.format_frame` raises
  instead.
- The NaN check covers accel x, y, z and gyro y only. gx and gz are NOT covered by any NaN
  check, so a non-finite gx or gz goes out as `0.000000` with a valid checksum.
- A non-finite accel x, y, z or gyro y calls `failSafe("imu nan")` and returns. No frame is
  sent and `seq` does not advance.
- A non-finite fused pitch calls `failSafe("pitch nan")` and returns. This is a second early
  return with no frame.
- `kFailMax` consecutive failed IMU reads call `failSafe("imu read failed")`.
- `kFailMax` consecutive failed `setGoalPosition` calls call `failSafe("dxl lost")`.
- Each `failSafe` turns the servo torque OFF (up to 3 tries) and LATCHES it until reset. The
  code comment says "Latched: torque off, stays off until reset". On a bench run, one NaN or
  a run of failed servo writes therefore drops torque for the rest of the run. A
  `[safe] torque off ...` line is printed. The `[safe]` line is printed only when torque was
  on. UNVERIFIED on hardware.
- After a latch, the loop keeps running, but only servo writes stop. The frame block is NOT
  gated on torque. So frames and the `[frames]` lines KEEP going out after a failSafe latch.

## Rate, size and back-pressure

- Nominal rate is 50 Hz (`kLoopUs` = 20000). A frame is attempted once per good IMU read.
- A frame is dropped when the TX space is short (see the next bullet).
- Back-pressure: a frame is sent only when the whole frame fits in the free
  `Serial.availableForWrite()` space. Otherwise the frame is DROPPED, not blocked, and
  `dropped` counts it.
- `seq` advances on every attempted frame, sent or dropped, so a gap in `seq` shows drops.
- A skipped IMU read (early return) leaves no `seq` gap. It shows only as a gap in `t_us`.
- Frame size: the 65-95 B figure is an ESTIMATE, not a measurement. The computed maximum is
  about 90 B (5-digit seq, 10-digit t_us, values up to +-4 g and +-500 dps), so 95 B is a
  loose bound. The derived upper rate is about 4.5 KB/s at 50 Hz. The 3.3-4.8 KB/s range is
  also derived from the estimate and is NOT measured. The ttl_servo Wokwi run saw 71-74 B
  frames (simulator output only; see #160 RESULTS).
- Wrap: `seq` is uint16 and wraps after 65536 frames (about 22 min at 50 Hz).
- Wrap: `t_us` is a 32-bit unsigned `micros()` value and wraps after about 71.6 min. The FW
  README "Frame protocol" text says `t_us` is capped at 2^32-1. These two statements differ.
  The FW has mock tests for a `t_us` wrap (`test_still_seq_and_t_us_wrap`), NOT run here. How
  the host treats a real wrap on a long run is UNVERIFIED. No host behaviour is claimed here.
- The 1 Hz line `[frames] sent=.. dropped=.. max_emit_us=..` prints only when
  `kFramesLineRoom` (80 bytes) of TX space is free. The worst case line is 67 bytes. If there
  is no room, the counters keep accumulating and the line is retried later.
- `max_emit_us` is a firmware-clock measure of format plus write time. No real-C3 value
  exists.
- Blocking risk (documented, NOT fixed): the unguarded 10 Hz `[imu]` `printf` and other text
  output can still block if the host stalls or is absent. No `setTxTimeoutMs` is set. A
  comment in `main.cpp` says the OLED push blocks about 25 ms. That figure was NOT measured
  here.

## Sharing the port

- The text lines `[tilt]`, `[imu]`, `[dxl]`, `[frames]` and `[safe]` share the USB CDC port
  with the frames. A consumer must filter on the `IMU,` prefix. The FW monitor and HAL do this.
- Only one process can hold the CDC port.
- The FW bridge drops text lines silently. It does not show the `[frames]` counters or
  `fused=`. Use two runs: first `pio device monitor` with `tee` for the counters and `fused=`.
  Then close it and run the bridge with `--record`.
- Per the FW README, opening the port toggles DTR/RTS and resets the ESP32. On `tilt_servo`,
  that re-runs `setup()` and `servoStart()` (torque on, goal = present position). UNVERIFIED
  for this firmware. Keep the servo secured or absent.

## Host side (from the FW code)

FW README section "Wheel bridge (dof_wheel_bridge, #375)", label "Behaviour". The arm window,
span gate and bias/tare ownership are from `dof_hal.py` `SerialImuHal` at FW `ab3c4f4`, read
from the merged file, not re-run.

- The bridge (`SerialImuHal`) arms on about 1 s (50 frames) of stillness, with a gyro span of
  5 dps or less. It then owns the bias (mean gyro) and the tare (mean fused pitch).
- `tilt_servo` sends RAW values, so a RAW stream is the right input. There is no double bias
  correction.
- The host never writes to the board.
- `yaw` is always 0.0 in the bridge output, by design.

## Capture and replay

FW README section "Wheel bridge (dof_wheel_bridge, #375)", label "Two CSV formats".

- `--replay` takes a RAW CSV with the header `seq,t_us,ax,ay,az,gx,gy,gz` and at least 50
  frames.
- It is NOT the `--record` output format, and it is NOT a text log.
- The conversion from a text capture to the raw CSV is a `sed` one-liner plus a header line.
  It was RUN on the ttl_servo Wokwi capture only. NOT run on a `tilt_servo` capture.
- A frame that the Wokwi CLI split with its own log lines fails the regex. See #160 RESULTS
  for the cause.

```bash
printf 'seq,t_us,ax,ay,az,gx,gy,gz\n' > raw.csv                                            # RUN (ttl_servo capture, header line); NOT run (tilt_servo)
sed -n 's/^IMU,\([^*]*\)\*[0-9A-F][0-9A-F]\r\?$/\1/p' capture.log >> raw.csv               # RUN (ttl_servo capture); NOT run (tilt_servo)
```

- The one-liner does NOT verify checksums, because the replay recomputes them. To check a
  capture, parse it with the FW `dof_frame.parse_line`.
- Result of the ttl_servo capture conversion (RUN): 299 rows from the 300 captured frames. The
  main session observed that the converted CSV's first row has `seq` 1. #160 RESULTS record
  that one of the 300 frames is split in the capture by the Wokwi CLI's own log lines. #160
  does not name `seq` 0.
- Bridge replay, RUN on the ttl_servo capture, with the context below. Both runs exited 0 with
  0 faults. Each run gave 250 armed ticks, 250 UDP datagrams on loopback port 5101 and 250
  record rows. 49 rows were consumed before arming. The FW README says the arm window is about
  50 frames (about 1 s). Replay result, checked by the main session: `roll`, `pitch`, `yaw`,
  `wheel_angle` and `cmd_rad_s` are exactly 0.0 in all 250 rows of both record files. `seq`
  runs 0..249 and `t_us` is non-zero. That stream only moves gz.

Context for the replay runs (RUN with adjusted paths): the main session ran these from an
export (`git archive`) of freecad-workspace `origin/main` at `ab3c4f4`, placed in a scratch
directory. The commands below use absolute paths, with a separate `--record` file per run
(`out_none.csv`, `out_lqr.csv`). The relative one-file form shown in the FW README
(`--replay raw.csv --record out.csv`) was NOT run literally. `<scratch>` stands for the
scratch directory.

```bash
mamba run -n esp32-dof python3 <scratch>/fw/poc/esp32-dof/monitor/dof_wheel_bridge.py --replay <scratch>/raw.csv --core none --record <scratch>/out_none.csv --udp-port 5101   # RUN with adjusted paths (see note above)
mamba run -n esp32-dof python3 <scratch>/fw/poc/esp32-dof/monitor/dof_wheel_bridge.py --replay <scratch>/raw.csv --core lqr --record <scratch>/out_lqr.csv --udp-port 5101    # RUN with adjusted paths (see note above)
```

This checks parse, arm and UDP plumbing only. It says nothing about motion, direction or
signs. The sim and replay results on this page are for the ttl_servo POC stream only, never
the tilt_servo stream.

NOT run: `--port` (serial), port 5006, Webots, and any `tilt_servo` stream.

## Human run steps

Pass criteria live in #159 and FW #377. This is a short ordered list only.

1. Run during the same bench session as the #84 checks. #84 is the open human issue on
   FE-URT-1 + XL330 hardware parity.
2. Keep the servo secured or absent. A `[safe]` line means torque is off until reset.
3. Build and upload with the flag. The upload command must carry the flag too. NOT run here.
4. `pio device monitor -e tilt_servo-xiao_c3 | tee capture.log` (NOT run). Only one process
   can hold the USB port, and opening the port resets the board (per the FW README;
   UNVERIFIED for this firmware, so keep the servo secured or absent). Record `max_emit_us`
   and `dropped`. Confirm the `[imu]` and `[dxl]` lines still appear. If any `[safe]` line
   appears, record it.
5. Close the monitor.
6. Run the bridge with `--record` (see the FW README "Wheel bridge" section). NOT run here.
   Only one process can hold the USB port, so the monitor must be closed first. Opening the
   port resets the board again (UNVERIFIED for this firmware; keep the servo secured or
   absent).
7. Compare pitch deltas, not absolute values, with `fused=`. Note any sign flip. Do NOT fix
   anything during the run.
8. Post RESULTS on #159. State what was not checked.

## Known UNVERIFIED items

- Everything on hardware: the stream, the servo behaviour and the board.
- `GYRO_PITCH_SIGN` (firmware) and the host `PITCH_SIGN` and `WHEEL_DISPLAY_SIGN`.
- DLPF lag compared with the POC (21 Hz here, 44 Hz in the POC).
- USB CDC write behaviour with no host attached.
- Soft-float formatting cost on the C3.
- Reset on port open (the DTR/RTS reset re-runs `servoStart()`).
- OLED-induced dt jitter.
- Bridge behaviour with a real `tilt_servo` stream.
- The frame size estimate and the derived KB/s figure.
- Host treatment of a real `t_us` wrap on a long run.

## ttl_servo POC (SIM-ONLY, disposable)

- `-D TTL_SERVO_IMU_FRAMES` in `programs/ttl_servo` is a SIM-ONLY disposable POC (#160). It
  is kept for now (#166).
- Its gyro range is +-250 dps. The accel range is the library default, 2 g.
- Scenario: `programs/ttl_servo/scenario_imu_frames.yaml`.
- The S5 result is quoted from #160 and was NOT re-run here. It reports 300 frames, all
  checksums valid under the real FW `parse_line`, `seq` 0..299, gz 0 to 30 deg/s, and
  `dropped=0`. The fall-test and replay addenda are on #160.
- That stream only moves gz. In the replay through the bridge, `roll`, `pitch`, `yaw`,
  `wheel_angle` and `cmd_rad_s` are exactly 0.0 in all 250 rows of both record files (checked
  by the main session); `seq` runs 0..249 and `t_us` is non-zero.
