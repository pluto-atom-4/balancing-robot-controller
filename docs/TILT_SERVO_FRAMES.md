# tilt_servo IMU frame stream (opt-in)

> **Status: opt-in diagnostic stream, NOT hardware-validated.** The `tilt_servo` frame stream has NOT been run in any simulator or on hardware. This page is written against `balancing-robot-controller` `main` 8e9d698 and the freecad-workspace (FW) merge `ab3c4f4` (PR #382). The FW docs may change (FW #376 is open).

Nothing on this page is a balance result. Every command below is marked **RUN** (executed for this page, with the result stated) or **NOT run**.

## Overview and cross-links

This page covers only what is specific to `tilt_servo`. It does not copy the frame format, arming rules or bridge usage. Those are owned by the FW README, `poc/esp32-dof/README.md` in freecad-workspace, sections "Frame protocol", "Monitor" and "Wheel bridge (dof_wheel_bridge, #375)", as read from the merged FW repo at merge `ab3c4f4` (PR #382).

Related tracking:

- FW #376: docs, open.
- FW #377: human GUI check, open.
- FW #387: optional bridge option to echo or log text lines, open.
- Local format source: `lib/imu_frame/imu_frame.h`.
- In this repo: #157 (the stream, merged), #172 and #173 (free-space gate, merged), #159 (human bench run), #160 (ttl_servo POC tracker).

Frame line shape (see the FW README for the full specification):

```
IMU,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>*HH
```

## Flag and build commands

The stream is compiled only when `-D TILT_SERVO_IMU_FRAMES` is set, through `PLATFORMIO_BUILD_FLAGS`. The default is OFF.

```bash
PLATFORMIO_BUILD_FLAGS="-D TILT_SERVO_IMU_FRAMES" pio run -e tilt_servo-xiao_c3   # RUN: success, no warnings (main session, 8e9d698)
pio run -e tilt_servo-xiao_c3                                                     # RUN: success (default build, main session, 8e9d698)
PLATFORMIO_BUILD_FLAGS="-D TILT_SERVO_IMU_FRAMES" pio run -e tilt_servo-xiao_c3 -t upload   # NOT run
pio device monitor -e tilt_servo-xiao_c3                                          # NOT run
```

The flag must also be on the upload command. Otherwise `pio` rebuilds the default firmware, and no frames appear.

Bridge commands are not repeated here. For the bridge, see the FW README section "Wheel bridge (dof_wheel_bridge, #375)". NOT run for `tilt_servo`.

## Cost (measured on 8e9d698)

Build sizes from the main session (RUN, builds only):

| Build | RAM | Flash |
| --- | --- | --- |
| default | 14680 B | 312092 B |
| `-D TILT_SERVO_IMU_FRAMES` | 14832 B | 313476 B |
| difference | +152 B | +1384 B |

A size match is not a behaviour claim. These numbers say nothing about timing, jitter or the stream's effect on the loop.

## What is sent

Checked against `programs/tilt_servo/main.cpp`:

- The values are RAW. Accel is sent in g and gyro in deg/s, converted from the Adafruit library units before formatting.
- Ranges: accel +-4 g, gyro +-500 dps. DLPF is 21 Hz.
- No boot-time bias removal and no filter.
- The firmware sign `GYRO_PITCH_SIGN` is NOT applied to the raw gyro y value in the frame.
- `t_us` is the loop-start `micros()` value, not the IMU sample time.
- The POC firmware `esp32_dof.ino` differs: about 1 s of boot gyro bias calibration, DLPF 44 Hz, and a sample-rate divisor of 9. Scales and noise therefore differ from the POC. The effect is UNMEASURED.
- The C++ formatter maps NaN and inf to 0.0. The Python `dof_frame.format_frame` raises instead.
- The NaN check covers accel x, y, z and gyro y only. gx and gz are NOT covered by any NaN check, so a non-finite gx or gz goes out as `0.000000` with a valid checksum.
- A non-finite accel or gyro y value returns early from the loop: no frame is sent and `seq` does not advance.

## Rate, size and back-pressure

- Nominal rate is 50 Hz (`kLoopUs` = 20000). One frame is sent per good IMU read.
- The 65-95 B frame size is an ESTIMATE, not a measurement. The 3.3-4.8 KB/s figure is derived from that estimate and is NOT measured. A ttl_servo Wokwi run saw 71-74 B frames; that is simulator output only.
- Back-pressure: a frame is sent only when the whole frame fits in the free `Serial.availableForWrite()` space. Otherwise the frame is DROPPED, not blocked, and `dropped` counts it.
- `seq` advances on every attempted frame, sent or dropped, so a gap in `seq` shows drops.
- A skipped IMU read (early return) leaves no `seq` gap. It shows only as a gap in `t_us`.
- The 1 Hz line `[frames] sent=.. dropped=.. max_emit_us=..` prints only when `kFramesLineRoom` (80 bytes) of TX space is free. The worst case line is 67 bytes. If there is no room, the counters keep accumulating and the line is retried later.
- `max_emit_us` is a firmware-clock measure of format plus write time. No real-C3 value exists.
- Blocking risk (documented, NOT fixed): the unguarded 10 Hz `[imu]` `printf` and other text output can still block if the host stalls or is absent. No `setTxTimeoutMs` is set. A comment in `main.cpp` says the OLED push blocks about 25 ms; that figure was NOT measured here.

## Sharing the port

- The text lines `[tilt]`, `[imu]`, `[dxl]`, `[frames]` and `[safe]` share the USB CDC port with the frames. A consumer must filter on the `IMU,` prefix. The FW monitor and HAL do this.
- Only one process can hold the CDC port.
- The FW bridge drops text lines silently, so it does not show the `[frames]` counters or `fused=`. Use two runs: first `pio device monitor` with `tee` for the counters and `fused=`, then close it and run the bridge with `--record`.
- Per the FW README, opening the port toggles DTR/RTS and resets the ESP32. On `tilt_servo`, that re-runs `setup()` and `servoStart()` (torque on, goal = present position). UNVERIFIED for this firmware. Keep the servo secured or absent.

## Host side (from the FW code)

- The bridge (`SerialImuHal`) arms on about 1 s (50 frames) of stillness, with a gyro span of 5 dps or less. It then owns the bias (mean gyro) and the tare (mean fused pitch).
- `tilt_servo` sends RAW values, so a RAW stream is the right input. There is no double bias correction.
- The host never writes to the board.
- `yaw` is always 0.0 in the bridge output, by design.

## Capture and replay

- `--replay` takes a RAW CSV with the header `seq,t_us,ax,ay,az,gx,gy,gz` and at least 50 frames.
- It is NOT the `--record` output format, and it is NOT a text log.
- The conversion from a text capture to the raw CSV is a `sed` one-liner plus a header line. RUN on the ttl_servo Wokwi capture only: 299 rows from 300 frames. A frame that the Wokwi CLI split with its own log lines fails the regex. NOT run on a `tilt_servo` capture.

```bash
printf 'seq,t_us,ax,ay,az,gx,gy,gz\n' > raw.csv                                            # RUN (ttl_servo capture, header line)
sed -n 's/^IMU,\([^*]*\)\*[0-9A-F][0-9A-F]\r\?$/\1/p' capture.log >> raw.csv               # RUN (ttl_servo capture); NOT run (tilt_servo)
```

- The one-liner does NOT verify checksums, because the replay recomputes them. To check a capture, parse it with the FW `dof_frame.parse_line`.
- Bridge replay, RUN on the ttl_servo capture. Both runs exited 0 with 0 faults, 250 UDP datagrams received on loopback port 5101, and 250 record rows. Every published value was 0.0, because that stream only moves gz.

```bash
mamba run -n esp32-dof python3 monitor/dof_wheel_bridge.py --replay raw.csv --core none --record out.csv --udp-port 5101   # RUN (ttl_servo capture): exit 0, 0 faults, 250 datagrams, 250 rows
mamba run -n esp32-dof python3 monitor/dof_wheel_bridge.py --replay raw.csv --core lqr --record out.csv --udp-port 5101    # RUN (ttl_servo capture): exit 0, 0 faults, 250 datagrams, 250 rows
```

This checks parse, arm and UDP plumbing only. It says nothing about motion, direction or signs.

NOT run: `--port` (serial), port 5006, Webots, and any `tilt_servo` stream.

## Human run steps

Pass criteria live in #159 and FW #377. This is a short ordered list only.

1. Run in the same session as #84.
2. Keep the servo secured or absent.
3. Build and upload with the flag (the upload command must carry the flag too). NOT run by the author of this page.
4. Run `pio device monitor -e tilt_servo-xiao_c3 | tee capture.log`. Record `max_emit_us` and `dropped`, and confirm the `[imu]`, `[safe]` and `[dxl]` lines still appear.
5. Close the monitor.
6. Run the bridge with `--record` (see the FW README "Wheel bridge" section).
7. Compare pitch deltas, not absolute values, with `fused=`. Note any sign flip. Do NOT fix anything during the run.
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
- The 4th `FELL` edge noted on #160 is out of scope here.

## ttl_servo POC (SIM-ONLY, disposable)

- `-D TTL_SERVO_IMU_FRAMES` in `programs/ttl_servo` is a SIM-ONLY disposable POC (#160). It is kept for now (#166).
- Its gyro range is +-250 dps. The accel range is the library default, 2 g.
- Scenario: `programs/ttl_servo/scenario_imu_frames.yaml`.
- The S5 result is quoted from #160 and was NOT re-run here: 300 frames, all checksums valid under the real FW `parse_line`, `seq` 0..299, gz 0 to 30 deg/s, `dropped=0`. The fall-test and replay addenda are on #160.
- That stream only moves gz, so a replay through the bridge shows pitch about 0.
