# Hardware-in-the-loop checklist: `balance-xiao_c3`

> **Status: DRAFT. NOT VALIDATED.** Nothing in this file is validated until a human runs it on the
> real board and fills in the boxes. Agents never flash, run, probe, or tick a box, and never fill
> the sign-off block. Drafted 2026-10-27 from the issues and repo files cited below. Not run on hardware.

Tracking: `pluto-atom-4/balancing-robot-controller#34` (this checklist, part of
`pluto-atom-4/balancing-robot-controller#22`). It replaces `pluto-atom-4/freecad-workspace#352`
(F.14). Results go to the Stage F parent `pluto-atom-4/freecad-workspace#338` (see "Where results go").
The checklist depends on the `c3_facts` results of `pluto-atom-4/balancing-robot-controller#44`
(step 4). The 2026-10-05 comment on `pluto-atom-4/balancing-robot-controller#34` records that the human
has the hardware and signs the checklist. The body's "step 10" (result table and sign-off lines) maps to
the "Results template" and "Sign-off" sections of this file.

## Purpose and scope

This is the manual checklist a human follows to run the balance firmware on a real Seeed XIAO
ESP32-C3 with the real MPU-6050 and real Dynamixel XL330 wheel servos. It checks what the simulation
cannot: the servo bus, the IMU, the gyro bias, the real sign conventions, the real loop timing and the
kill paths.

- In scope: the `balance-xiao_c3` env (LQR) and the `balance-xiao_c3-pid` env (PID) from `platformio.ini`.
  XL330 servos on the bus through the FE-URT-1 adapter on `Serial1` (GPIO20 RX, GPIO21 TX). MPU-6050 on
  I2C (SDA = D4, SCL = D5).
- Not in scope: Feetech STS3032 (planned, untested); the final robot power path
  (`pluto-atom-4/balancing-robot-controller#178`, deferred); Webots; ESP32-S3; a BOOT-button (GPIO9)
  hardware kill (a TODO in `programs/balance/config.h:49`, not implemented, so not part of this run).

**Honesty rule.** Each constant, unit and baud here is either read from the repo (file and line in
brackets) or marked UNVERIFIED. A box is PASS only when the human has seen the observation on the real
hardware and written it down. "NOT CHECKED" is a valid result, and "DEFERRED" is valid only where the vocabulary below allows it. No step says that the
baud, the servo IDs, the velocity unit, the velocity limit, the PID or LQR sign, or the STS3032 is verified.

**Result vocabulary.** Each box is `PASS`, `FAIL` or `NOT CHECKED`. `DEFERRED` appears ONLY in step 5
(PID sign) and in the sign-off block. A FAIL is a result: record it and do not work around it.

## Inputs and their status

Values below were read from `programs/balance/config.h` (line in brackets). Every row is UNVERIFIED on
hardware or is a sim-model value; none is a C3 measurement.

| Input | Value | Status |
|---|---|---|
| Timing budget (`pluto-atom-4/freecad-workspace#345`, `jitter_budget.py`) | nominal 20 ms; sustained mean limit 21.5 ms; up to 3 consecutive missed updates; jitter +-9 ms [config.h:28-31] | Sim-model tolerance, NOT a C3 measurement |
| Over-budget threshold `kJitterBudgetUs` | 29000 us (20 ms + 9 ms) [config.h:32-34] | Sim-derived, UNVERIFIED on C3; TODO(human) |
| Servo baud `kServoBaud` | 57600 [config.h:11] | UNVERIFIED on C3 + FE-URT-1. config.h:11 labels it PROVEN; that label is not a bench result, see 4b |
| Servo baud requested `kServoBaudRequested` | 2,000,000 [config.h:12] | UNTESTED on C3 + FE-URT-1. Not used by the balance firmware. c3_facts tries it at boot (step 4a), but the servos are not set to 2,000,000; open decision (d) |
| Tilt cutoff `kTiltCutoffRad` | 0.5 rad [config.h:38] | UNVERIFIED |
| Startup lean gate `kStartupLeanGateRad` / `kLeanGateHoldTicks` | 0.1 rad about the centre (0 for PID, `THETA_REF` = -0.1086 rad for LQR), held 25 ticks (0.5 s at 50 Hz) [config.h:41-42; main.cpp:35, 40] | UNVERIFIED (SIM analogue) |
| Stall threshold `kStallUs` | 100000 us (100 ms) [config.h:37] | UNVERIFIED |

## Safety gates (read before any step)

- **G1. Wheels OFF the ground** for steps 2, 3 and 5 to 8 (robot on a stand, wheels free).
  Step 4 has a stricter condition (G1b).
- **G1b. Step 4 (4a and 4c): servo OFF the robot, HELD DOWN, NO wheel attached.** The c3_facts
  prompt says "servo must be OFF the robot, HELD DOWN, no wheel attached" [programs/c3_facts/main.cpp:2
  and the ramp prompt in offerRamp()]. "Wheels off the ground" is not the same condition. The physical
  setup changes between steps 3 and 4 and again before step 5.
- **G2. Servo supply separate and current-limited**, common GND with the FE-URT-1 and the XIAO
  [pluto-atom-4/balancing-robot-controller#34 step 1]. The supply limit value is the human's to set and
  record in step 1a.
- **G3. Clear area.** Nobody's hands are near the wheels or horns while the servos are powered.
- **G4. Kill check before any motion, including step 4c.** The minimal serial `x` check (step 1b) must
  pass before any lean test (steps 5, 6) and before the step 4c ramp. In 4c the serial abort is the
  c3_facts ramp's own (any byte aborts the ramp, lib/ramp_guard), not the balance latch.
- **G5. No EEPROM writes by agents or probes.** The balance firmware itself writes EEPROM: Operating
  Mode (register 11) at bring-up, up to two conditional writes (one per servo ID), each only when that
  servo is not already in velocity mode [programs/balance/dxl_wheels.h:81-87; lib/wheel_servo/wheel_servo.h].
  That is the firmware's write, not an agent's or probe's; see step 2 and open decision (e). Forbidden in
  this checklist, for agents and as a step for the human here: any baud, ID or Velocity Limit write; the
  c3_facts build flag `-D C3FACTS_ALLOW_MODE_WRITE` [programs/c3_facts/main.cpp:341]; any Dynamixel
  Wizard EEPROM change (a manual human action, not part of this run; see open decision (d)).
- **G6. No autonomous hardware action.** The human runs every flash, monitor and lean, with the command shown.
- **G7. One USB port user at a time.** Close `pio device monitor` before another tool opens the port
  [pluto-atom-4/balancing-robot-controller#159].
- **G8. No hot-unplug of a powered servo bus.** `pluto-atom-4/balancing-robot-controller#148` has no
  design chosen: the owner's 2026-10-21 Q1 answer is software-only for the bench, and the power cut is
  split to `pluto-atom-4/balancing-robot-controller#178`. The owner DECIDED 2026-10-27 (open decision (b)): switch the servo
  supply OFF instead of unplugging a powered bus.
- **G9. Do not disable a firmware kill path** to get a test through. Changing `programs/balance/config.h`
  or a kill path needs a design issue [CLAUDE.md, "Balance safety design"].

Commands (from `CLAUDE.md`, "Development Commands" and "Serial Debugging"): the upload form is
`pio run -e <env> -t upload`; the monitor form is `pio device monitor -e <env>` at 115200 baud. The
`| tee <file>` capture pattern is from `pluto-atom-4/balancing-robot-controller#84`. Do not use the
`blink-uno` example as the balance form.

---

## Step 1. Safety setup and minimal kill check

Note: step 1b runs in the step 2 session, after the firmware exists (see step 2).

**Precondition.** Gates G1 to G9 read.

**1a. Physical setup (before power).** Record G1 (wheels off the ground), G2 (servo supply
current-limited, common GND; supply limit set to: ____ A, the human's value), and G3 (clear area).

**1b. Minimal serial kill check (runs in the step 2 session).** This needs the firmware, so it runs after
step 2 has flashed it: in the step 2 monitor session, after the boot capture is saved and before any lean.
Type `x` in the monitor.

**Expected.** The log shows `[safe] LATCHED cause=serial` then `[safe] torque off confirmed`
(`bsup::name` prints `serial` for SerialKill [lib/balance_supervisor/balance_supervisor.h:158];
[programs/balance/main.cpp:122-127]). The latch is terminal: reset the board to re-arm (there is no
re-arm command). If `[safe] TORQUE OFF NOT CONFIRMED` appears, power the servo supply OFF at once (decided 2026-10-27, open decision (b)) and record it.

**Record.** 1a: `PASS` / `FAIL` / `NOT CHECKED`, notes: ____.
1b: `PASS` / `FAIL` / `NOT CHECKED`, paste the latch and torque lines: ____.

**Unverified.** The torque-off behaviour on this hardware.

---

## Step 2. Flash the firmware and capture the boot log

**Precondition.** Step 1a PASS. Wheels off the ground (G1). Servo supply ON, current-limited. Robot still.
**Owner decision (e) is accepted by running this step.** At bring-up the firmware writes Operating Mode
(register 11) on each servo ID: up to two conditional EEPROM writes [programs/balance/dxl_wheels.h:81-87].
Running this step is the acceptance; see open decision (e).

**Action.**
```bash
cd ~/Documents/platform-io-workspace/balancing-robot-controller
pio run -e balance-xiao_c3 -t upload
pio device monitor -e balance-xiao_c3 | tee balance_boot.log
```
Reset the board and keep the robot still. Save the boot capture, close the monitor, then run step 1b in
the same step 2 session (a new monitor session is fine), then reset the board.

**Expected lines** (from `programs/balance/main.cpp`):
- `[bal] boot LQR (control loop, UNVERIFIED on hardware)`. The `PID` env prints `PID` here.
- `[wheel] XL330 ready baud=57600 ids=1/2 torque on, goal 0 (UNVERIFIED on hardware)` [main.cpp:79-81].
  `XL330` is the family from `dxl_wheels.h` (`family()` returns "XL330"). The `baud=57600` value is the
  CONFIGURED constant `kServoBaud` [config.h:11], printed on success. It is not a measurement and cannot
  support a baud observation. Baud is recorded in step 4b.
- `[imu] ready`, `[imu] calibrating gyro bias, keep robot still`, then
  `[imu] bias ok y=<value> rad/s (UNVERIFIED on hardware)` [main.cpp:90-93]. The firmware prints the Y axis only.
- `[bal] LQR waiting for lean gate (|pitch-center|<=0.100 rad)` [main.cpp:109-110].

**Servo ping, both IDs.** Bring-up pings ID 1 then ID 2 and fails if either fails
[lib/wheel_servo/wheel_servo.h:77-80]. The firmware does not print per-ID answers.
- Success: the `[wheel] XL330 ready ... ids=1/2` line.
- A missing ID (ID 1 or ID 2): expected text, all from code paths: `[wheel] startup FAILED fault=4, torque off,
  read-only (TORQUE OFF NOT CONFIRMED)`. Fault 4 is `TorqueOffUnconfirmed` [lib/wheel_servo/wheel_servo.h:15].
  The missing ID cannot confirm torque off, so the fault is reported as 4 and not as 1 (`NoPing`).
  Then `[safe] LATCHED cause=startup` [main.cpp:122-123]. Then `[safe] TORQUE OFF NOT CONFIRMED, retrying`
  one second apart, up to 5 tries [main.cpp:44-47, 128-131], and possibly `[safe] TORQUE OFF NOT CONFIRMED,
  gave up: REMOVE SERVO POWER`. Record every line. A missing ID is a RESULT, not a reason to change `config.h`.

**Expected EEPROM write (not printed).** Up to two Operating Mode (register 11) writes happen at bring-up,
one per servo ID, each only if that servo is not already in velocity mode [programs/balance/dxl_wheels.h:81-87].
No log line shows them. Operating Mode is NOT read before step 2 in this checklist (4a runs once, after
step 3), so whether the write happened is NOT CHECKED; record the Operating Mode read in 4a. This is open decision (e).

**Record.** Paste the boot lines. ID 1 answered / not answered / unknown. ID 2 answered / not answered /
unknown. Gyro bias y: ____ rad/s.
- `PASS` only if the `[wheel] XL330 ready ... ids=1/2` line, `[imu] ready` and `[imu] bias ok` all appear.
- `FAIL` otherwise (write the missing or faulted line).
- `NOT CHECKED` if the boot was not captured.

**Unverified.** ID 1 and ID 2 (`config.h:15-16`, "TODO(human)"). The servo family text is from code,
not from a probe on the bus.

---

## Step 3. Gyro bias, twice, with a power cycle

**Precondition.** Step 2 captured. Robot still and flat. Wheels off the ground (G1).

**Action.**
1. Keep the robot still for the whole calibration. Record value 1 from `[imu] bias ok y=`.
2. Power cycle with the robot still. Capture the boot again with the step 2 command into
   `balance_boot_2.log`. Record value 2.
3. Record the difference.

**Duration.** The issue says "still 5 s" [pluto-atom-4/balancing-robot-controller#34 step 3]. The code
takes 500 samples with `delay(2)` each [programs/balance/imu_mpu6050.h:48-59]: about 1 s plus I2C time.
The 5 s figure is the issue's figure and is UNVERIFIED against this code.

**Expected.** Both runs print `[imu] bias ok`. The two values agree. The repo sets no numeric limit for
"agree": the human judges it and writes the difference. Code limits, marked "UNVERIFIED guesses"
[imu_mpu6050.h:26-28]: |mean| <= 0.15 rad/s per axis and variance <= 4.0e-4 (rad/s)^2.

**Record.** Value 1: ____ rad/s. Value 2: ____ rad/s. Difference: ____.
- `PASS` both runs print `[imu] bias ok` and the human judges the values consistent.
- `FAIL` a run prints `[imu] bias FAILED`, or the values are not consistent (write why).
- `NOT CHECKED`.

**Retry behaviour.** After a bias FAILED, the firmware retries calibration every 3 s
[main.cpp:45, 219-227]. Repeated `[imu] calibrating gyro bias` and `[imu] bias FAILED` lines are expected.
If the IMU was never found at boot, the first boot prints `[imu] MPU6050 not found (check SDA=D4 SCL=D5, AD0)`
[main.cpp:210], and each retry prints `[imu] not found` [main.cpp:224]. Record the number of retries.

---

## Step 4. Wheel servo facts

**Precondition.** G1b: servo OFF the robot, HELD DOWN, NO wheel attached, for 4a and 4c. Steps 1, 2 and 3 done.
**Order (fixed): steps 1, 2 and 3, then 4a ONCE.** 4c runs after 4a, and only when Operating Mode is velocity.

**4a. Read the device (no motion).**
```bash
pio run -e c3_facts-xiao_c3 -t upload
pio device monitor -e c3_facts-xiao_c3 | tee c3facts.log
```
c3_facts turns torque OFF at boot and on every exit. Its boot tries 57600 and then 2,000,000 baud, pings
both IDs and sends a torque-off write to any servo that answers [programs/c3_facts/main.cpp:27, 51-56].
So 2,000,000-baud bus traffic does happen here, although the servos are not set to 2,000,000 (see 4b).
The keys include `servo1_ping` and `servo2_ping` (separate booleans; a missing ID is reported and is not
fatal) [main.cpp:234-237], `servo<N>_model_number` and `servo<N>_model_name` (1190 names XL330-M077
[programs/c3_facts/facts_io.h:12,16]), the Velocity Limit device value, Operating Mode, torque state and
present velocity at rest.

**Typed-`t` tilt phase: decline unless intended.** c3_facts offers an optional forward-tilt capture
(type `t` within 15 s; prompt: "tilt the body FORWARD about 20 deg") [programs/c3_facts/main.cpp:140-156].
Do NOT tilt the body while the servo is attached. Let the window expire unless you mean to run it.
If run, it prints `pitch_forward_*` and a sign hint. The IMU pitch sign is UNVERIFIED:
`kPitchSign` = 1.0 and `kGyroPitchSign` = 1.0 [programs/balance/imu_mpu6050.h:20-25], with the code's own
comment "if fused pitch moves opposite to accel pitch, flip". The sign hint is not a verification.

After 4a and 4c, flash the balance firmware again before step 5:
`pio run -e balance-xiao_c3 -t upload`.

**4b. Baud actually used.** The firmware uses 57600 (`kServoBaud`, `config.h:11`). That file labels 57600
"PROVEN" (from `tilt_servo/main.cpp:34`). CONFLICT: `CLAUDE.md` says `tilt_servo` is "not hardware-validated",
so the "PROVEN" label is the repo's label, not a bench result. The 57600 factory baud was stated by the human
in a 2026-10-06 comment on `pluto-atom-4/balancing-robot-controller#44`. Record the baud the servos ANSWERED
at, from 4a: `baud_used = ____`. 2,000,000 is requested and UNTESTED on C3 with FE-URT-1; testing it needs a
manual Dynamixel Wizard EEPROM change, which this checklist does not do (open decision (d)).

**4c. Velocity unit, limit, ramp, and left/right sign (commands motion; human opt-in).**
- Build: `PLATFORMIO_BUILD_FLAGS="-D C3FACTS_RAMP" pio run -e c3_facts-xiao_c3 -t upload`
  [pluto-atom-4/balancing-robot-controller#44 comment]. Run the monitor as in 4a.
- The ramp is refused (`ramp_refused` true, a recordable result, not a servo fault) unless ALL hold
  [programs/c3_facts/main.cpp:332-357; lib/ramp_guard]: the servo pings; the model reads 1190
  (XL330-M077); the Velocity Limit is read from the device; Operating Mode is already velocity (the
  mode-write flag is forbidden here, G5); the host is connected; torque off is confirmed by read-back.
  Normally Operating Mode is velocity only after step 2's boot write, so 4c runs after 4a (which follows steps 2 and 3).
- If it runs: the ramp switches torque ON (c3_facts `runRamp` calls `enableVelocityMode`
  [programs/c3_facts/main.cpp:287]). That is safe only with the servo held down. The cap is 20 percent of the
  device-read Velocity Limit, positive only, about 15 s, 20 s hard timeout. It needs a typed `GO` and Enter
  within 60 s [lib/ramp_guard/ramp_guard.h:19], then a 5 s countdown. Any key aborts. Read Present Velocity
  from `ramp_row` lines (`C3FACTS ramp_row <t_ms>,<goal>,<present> ms_raw_raw`).
- Velocity unit: `lib/dxl_units/dxl_units.h:10` says 0.229 rev/min per raw count, and the velocity
  limit range is 0 to 2047 [dxl_units.h:11]. The header cites the XL330-M077 e-Manual URL
  (`https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/`) but gives no page. Write the page you read.
  UNVERIFIED until you do.
- Velocity Limit: the repo's default is 1620 [dxl_units.h:32]. The issue `pluto-atom-4/balancing-robot-controller#34`
  body says 1023. Record the device value and do not assert either (open decision (c)).
- Left/right sign: the ramp drives ID 1 only [programs/c3_facts/main.cpp:332]. The right wheel (ID 2)
  velocity and the left/right sign are NOT CHECKED by any repo tool.

**Expected.** 4a: model name printed, `servo1_ping` true, `servo2_ping` as the device says (false is a
result). 4c: if the ramp runs, Present Velocity follows the sign of the positive goal; the unit match
against the e-Manual is for the human to record.

**Record.**
- 4a model / name: ____. Operating Mode: ____. Torque state: ____. Device Velocity Limit: ____.
  `servo1_ping`: ____. `servo2_ping`: ____. Present velocity at rest: ____. IMU pitch sign: ____.
  `PASS` / `FAIL` / `NOT CHECKED`.
- 4b baud used: ____. `PASS` only if an answering baud is recorded; `FAIL` if no baud answers;
  `NOT CHECKED` for 2,000,000.
- 4c `ramp_refused` true or false, end reason, velocity unit vs e-Manual page ____ (page required).
  `PASS` / `FAIL` / `NOT CHECKED`.
- 4c left/right sign: ID 1 by ramp (`PASS` / `FAIL` / `NOT CHECKED`); ID 2 `NOT CHECKED` unless a human checked it by hand.

**Unverified.** Model number (the device value counts; `facts_io.h:14` marks 1200 as UNVERIFIED and the
sim chip reports 1200 [wokwi/chips/wokwi-ttl-servo.c:8]). The velocity unit 0.229 and limit 1620 (open
decision (c)). IDs (step 2). The left/right sign (`config.h:17-18`, unconfirmed). The right-wheel velocity.

---

## Step 5. PID sign check (optional; may be DEFERRED)

**Precondition.** Step 4 done. Step 1b PASS. Wheels off the ground (G1). Robot on the stand.

**Action.**
```bash
pio run -e balance-xiao_c3-pid -t upload
pio device monitor -e balance-xiao_c3-pid | tee pid_lean.log
```
(Build form from `CLAUDE.md`, "Development Commands". The `-t upload` form is the same as the LQR env.)

**Centre and arming.** The PID centre is 0 [programs/balance/main.cpp:40]. The lean gate is
`|pitch| <= 0.100 rad` for 25 ticks [config.h:41-42]. If the rest pitch is outside +/-0.1 rad of 0, the
PID does NOT arm, and step 5 cannot finish. Record NOT CHECKED with the note `did not arm` and the observed pitch
value (from `[bal] pitch=` lines, 10 Hz).

**Action (if armed).** Once `[bal] PID ARMED (UNVERIFIED sign, wheels off ground)` appears, tilt the robot
slightly forward by hand, note which way the wheels turn, return upright, and tilt slightly back. Keep the
tilt small: the latch is above `kTiltCutoffRad` = 0.5 rad from the centre [config.h:38].

**Expected.** Record the observed direction of the wheels for a known forward lean. The two controllers
are documented to give OPPOSITE signs for the same tilt (UNVERIFIED; `pluto-atom-4/freecad-workspace#359`; `programs/balance/config.h:44-45`). The body of `pluto-atom-4/balancing-robot-controller#34` step 5 said
"wheels must drive toward the lean"; that expectation was intentionally removed from this file, pending the
owner's decision on the sign (`pluto-atom-4/freecad-workspace#359`). So the expected sign is not decided here,
and the observed direction is NOT CHECKED against any expectation.

**Record.** Forward lean: wheels turned ____. Back lean: wheels turned ____. Rest pitch ____ rad.
- `NOT CHECKED` sign vs expectation (the normal result until `pluto-atom-4/freecad-workspace#359` is decided), with the observed direction recorded.
- `FAIL` if the wheels do not respond at all, or the latch trips at a small lean (write the pitch).
- `DEFERRED` (reason: ____). Deferral is allowed for this step.
- `PASS` is NOT an allowed result for this step until the expected sign is decided (`pluto-atom-4/freecad-workspace#359`). Step 5 allows only FAIL, NOT CHECKED or DEFERRED.

**Unverified.** The PID sign, the PID gains (`kPidKp` = 1.0, `kPidKi` = 0.1, `kPidKd` = 0.05,
`lib/balance_core/balance_gains_generated.h:12`), and PID stability (`pluto-atom-4/freecad-workspace#338`
notes it untested).

---

## Step 6. LQR sanity check

**Precondition.** Step 1b PASS. Wheels off the ground (G1). Robot on the stand.

**Action.**
```bash
pio run -e balance-xiao_c3 -t upload
pio device monitor -e balance-xiao_c3 | tee lqr_lean.log
```
Hold the robot in the gate until `[bal] LQR ARMED (UNVERIFIED sign, wheels off ground)` appears. Then
record `[bal] pitch=... cmd=... state=... cause=... imu=...` lines (10 Hz) at rest, and at small steps
forward and back [programs/balance/main.cpp:184-189].

**Centre and arming.** The LQR centre is `kThetaRefRad` = -0.1086 rad (`balance_gains_generated.h:10`,
a SIM plant value). The lean gate is `|pitch - THETA_REF| <= 0.1` [main.cpp:35, config.h:41]. If the
real rest pitch is outside that band, the LQR does not arm: record NOT CHECKED with the note `did not arm` and the observed pitch.

**Expected.**
- The issue `pluto-atom-4/balancing-robot-controller#34` step 6 asks for command "proportional to tilt and
  gyro". This file checks the TILT part only: the command magnitude grows with the tilt magnitude. That is the PASS check.
  The GYRO part is moved to NOT CHECKED (see below).
- The command sign for a known forward lean is observed and recorded. Its expected sign is NOT decided
  here: the LQR and PID are documented to give opposite signs for the same tilt (UNVERIFIED; `pluto-atom-4/freecad-workspace#359`; `programs/balance/config.h:44-45`). Record, and mark the sign NOT CHECKED against any expectation.
- The robot settles near an equilibrium. Record the rest pitch. Compare it with -0.1086; do not assume equality.
- Gyro: the `[bal]` line does NOT print the gyro rate, so the gyro part cannot be checked from this log.
  It stays NOT CHECKED unless a human adds a print (no code change in this checklist).

**Record.** Rest pitch ____ rad (vs -0.1086: ____). Observed command for a forward lean: ____.
- `PASS` command magnitude grows with the tilt, and the rest pitch is recorded.
- `FAIL` the magnitude does not grow with the tilt (write the data).
- `NOT CHECKED` otherwise. Sign: NOT CHECKED vs expectation. Gyro part: NOT CHECKED.

**Unverified.** The LQR sign, the gains (`kLqrK` in the generated header), the equilibrium, the gyro proportionality.

---

## Step 7. Loop timing

**Precondition.** Step 3 done (gyro calibrated). Wheels off the ground (G1). Record the number of servos
connected (one or two).

**Step 7 rule: owner decision 2026-10-27.** The `[loop]` statistics are cumulative since calibration end and
include every control tick, including the lean-gate ticks before arming [programs/balance/main.cpp:101-104
reset; main.cpp:182 adds every tick]. Those pre-arm ticks write nothing to the servos (UNVERIFIED that they run at a different period: the period is not logged per state). The owner
decided to ALLOW them and to RECORD the dilution. The gate holds `kLeanGateHoldTicks` = 25 ticks (0.5 s at 50 Hz)
before arming [programs/balance/config.h:42]. At about 1500 samples that is about 1.7 percent of the samples,
so the mean may be slightly optimistic.

**Action.** Run the balance env (the step 6 session is fine). Arm promptly: hold the robot in the gate so that
`[bal] ... ARMED` follows the lean-gate line without a long wait. Once `state=Running`, leave it running at least
about 30 s. Then read the first `[loop]` line printed at least 30 s after arming. `[loop]` prints once per second
[main.cpp:44, 190-197]. The `[bal]` lines (10 Hz) show `state=` [main.cpp:184-189]; check that they read `Running`
on every line from arming through the reading. Record `n` from that `[loop]` line.
- If the robot was not Running continuously after arming (a re-arm, a latch, or a long wait in `WaitingForLean`
  before arming), the result is NOT CHECKED. Long pre-arm waits dilute the statistics much more than 25 ticks.
- Restart: reset the board, then arm promptly and take the window again from the new arming.
- Line format: `[loop] min=<us> mean=<us> max=<us> over=<n> n=<n> us`.

**Pass criteria** (exactly as in the 2026-10-07 comment of `pluto-atom-4/balancing-robot-controller#34`):
1. `mean` at most 21500 us. This is the sustained-mean limit from `pluto-atom-4/freecad-workspace#345`
   (the F.6 numbers, from the `hal/jitter_budget.py` docstring in `pluto-atom-4/freecad-workspace#345`,
   file `inverted-pendulum-project/07_Simulation/hal/jitter_budget.py` in `pluto-atom-4/freecad-workspace`).
   The `programs/balance/config.h:29-30` citation gives the same set of sim-model numbers (nominal 20 ms,
   sustained mean 21.5 ms, +/-9 ms jitter, 3 consecutive missed updates). This is a sim-model tolerance, not a C3 measurement.
2. `over` = 0. `over` counts single periods above `kJitterBudgetUs` = 29000 us [config.h:34].
3. Sanity only, not a gate: `n` about 1500 after about 30 s at 50 Hz.

**Two different checks.** 21500 us (mean) and 29000 us (`kJitterBudgetUs`, per-period "over") are
different checks. A loop steadily at 22 ms gives over = 0 and still fails the mean check. The 29000 us
value is 20 ms nominal plus 9 ms jitter; the repo marks that derivation TODO(human) [config.h:32-33].

**Record.** Servos connected: 1 / 2. Arming time: ____. `n` ____. min ____ us, mean ____ us, max ____ us,
over ____. Latched during the run: yes / no. Running on every `[bal]` line: yes / no.
The record MUST state: includes ~25 pre-arm ticks (about 1.7%); mean may be slightly optimistic.
- `PASS` Running on every `[bal]` line from arming through the reading, reading taken at least 30 s after arming,
  mean at most 21500 us AND over = 0.
- `FAIL` Running throughout, and either limit fails (write which).
- `NOT CHECKED` not Running continuously after arming (re-arm, latch, or long pre-arm wait), or the reading was
  taken less than 30 s after arming.

**Notes.** The retry bound on a latched run is a CODE bound, not a measurement. Per torque-off call, WheelPair
`offConfirmed()` calls `torqueOff` for both IDs [lib/wheel_servo/wheel_servo.h:116-121]. Each `torqueOff` makes
3 tries [programs/balance/dxl_wheels.h:69-77]; each try is one write and one read-back, and each I/O has a
20 ms timeout [dxl_wheels.h:21]. Worst case: 2 IDs x 3 tries x 2 I/O x 20 ms = 240 ms per torque-off call. The
latch path then retries up to 5 times, 1 s apart [main.cpp:44-47, 128-131]. The "about 240 ms" in the
2026-10-07 comment matches this code bound. It is UNVERIFIED as a measurement. Do not read a latched run as
a timing result. Real C3 numbers come from `pluto-atom-4/balancing-robot-controller#43` (`c3_probe`), not yet run.

---

## Step 8. Full kill-switch set

**Precondition.** The robot is armed, wheels off the ground (G1), robot on the stand. After each latch, reset the board and re-arm (wait for `ARMED`) before the next sub-check. The serial `x` check is NOT repeated here: it is step 1b (one result, not two; the `pluto-atom-4/balancing-robot-controller#34` body places the full kill set at step 8, see open decision (a)).

**8.1 Tilt cutoff.** Env: `balance-xiao_c3` (LQR), run as in step 6. Flash only if the board does not already run this env: `pio run -e balance-xiao_c3 -t upload`.
Monitor: `pio device monitor -e balance-xiao_c3 | tee tilt_cutoff.log` (forms from `CLAUDE.md`, "Development Commands"
and "Serial Debugging"). Tilt the robot past 0.5 rad from the centre (`kTiltCutoffRad`,
`config.h:38`), where the centre is THETA_REF for LQR and 0 for PID [main.cpp:35, 40]. Expected:
`[safe] LATCHED cause=tilt`, then `[safe] torque off confirmed`. Record both lines. `PASS` / `FAIL` / `NOT CHECKED`.

**8.2 IMU unplug.** Env: `balance-xiao_c3` (LQR). Flash only if the board does not already run this env:
`pio run -e balance-xiao_c3 -t upload`. Monitor: `pio device monitor -e balance-xiao_c3 | tee imu_unplug.log`
(forms from `CLAUDE.md`, "Development Commands" and "Serial Debugging"). Disconnect the MPU-6050 (SDA D4,
SCL D5) while armed. Expected: after `kImuFailMax` = 3 consecutive read failures [config.h:43],
`[safe] LATCHED cause=imu` and torque off. Record the lines. `PASS` / `FAIL` / `NOT CHECKED`.

**8.3 Servo failsafe.** Env: `balance-xiao_c3` (LQR), or `balance-xiao_c3-pid` if the human chose it in step 5.
Flash only if the board does not already run this env: `pio run -e <env> -t upload`. Monitor:
`pio device monitor -e <env> | tee servo_failsafe.log` (forms from `CLAUDE.md`, "Development Commands" and
"Serial Debugging").
Action DECIDED 2026-10-27 (open decision (b)): switch the servo supply OFF, not to
hot-unplug a powered bus (G8). Expected: after `kServoFailMax` = 3 consecutive write failures [config.h:23] the
latch is `cause=wheel` [lib/balance_supervisor/balance_supervisor.h:157; main.cpp:122-123], then the torque-off
lines as in step 2 (`TORQUE OFF NOT CONFIRMED, retrying` / `gave up: REMOVE SERVO POWER` possible). Record
every line. `PASS` / `FAIL` / `NOT CHECKED`.

**Unverified.** Every cause name and timing on this hardware. The servo-off method (decided 2026-10-27, (b)).

---

## Step 8b. OPTIONAL: XL330 bus watchdog (register 98)

**Status.** OPTIONAL, not required for sign-off (DECIDED 2026-10-27, open decision (f)). If run, record PASS / FAIL / NOT CHECKED. It was requested as a
hardware-checklist item by the owner's answer to `pluto-atom-4/balancing-robot-controller#148` Q3
(2026-10-21), and proposed on `pluto-atom-4/balancing-robot-controller#34` that day. It has not been run.

**Source (UNVERIFIED for this servo).** XL330-M077 e-Manual, register 98: 1 byte, RAM, range 1 to 127,
unit 20 ms, 0 = off. If no valid instruction arrives within the period while Torque Enable is 1, the servo
stops, reads -1, and goal writes return a Data Range Error until 0 is written. The manual page does not
say torque is disabled on timeout. The sim chip reports model 1200 (not the same model). Part of the page was not read.

**Precondition.** Servo OFF the robot, HELD DOWN, NO wheel attached (the same condition as G1b, because
this step commands a goal velocity). Separate, current-limited supply, with common GND as in the 2026-10-21 comment of `pluto-atom-4/balancing-robot-controller#34`
(that comment does not say what the common ground connects to; the human confirms the wiring). A Dynamixel Wizard 2.0
session (human-operated; reads and writes single registers). NO balance firmware and NO control loop running.

**Items** (each `PASS` / `FAIL` / `NOT CHECKED`):
1. Register 98 reads 0 at power-up.
2. Write a non-zero value (record value and unit), enable torque, command a small goal velocity, send nothing.
   Record whether the servo stops, how long after the last instruction.
3. After timeout: read Torque Enable, Present Velocity, register 98 (expected -1 per the manual), and a goal write's error.
4. Clear by writing 0 to register 98. Record what else is needed to recover.
5. Refresh test: with a value set, send a valid instruction faster than the period for 30 s. The servo must not stop.
6. Record which instructions reset the timer (read, write, ping).

**Result rule.** A pass does NOT make the watchdog a safety layer by itself. The owner decides its use
(`pluto-atom-4/balancing-robot-controller#148` option D, with the software kill paths kept). No firmware
kill path or `config.h` constant changes on this test's strength.

---

## Step 9. Supervised free-balance attempt (optional)

**No written procedure exists.** Human-supervised only. Skip unless the owner adds one.

**Precondition (if run).** Steps 1 to 8 are PASS, except step 5, which follows its own step 5 result rule (see step 5).
If any other step is not PASS, do not run this step. A
written kill plan (serial `x`, tilt cutoff, servo supply OFF as the last resort). A human present for the whole run.

**Record.** `PASS` / `FAIL` / `NOT CHECKED`. Notes: ____. No repo number defines a successful free balance.

---

## Results template

Post the filled template as a results comment (see "Where results go"). Do not fill it in this file.

- Date: ____
- Board: Seeed XIAO ESP32-C3 (label: ____)
- Env: `balance-xiao_c3` / `balance-xiao_c3-pid` (circle one). Build flags: ____
- Firmware SHA (BRC commit): ____
- Gains hash (`kGainsHash` in `lib/balance_core/balance_gains_generated.h` as built; 0x1e60b0fc at BRC
  commit 17436fe): ____
- Servos connected: 1 / 2. Baud that answered (4b): ____. Supply limit (1a): ____
- Per step, one line each, with `PASS` / `FAIL` / `NOT CHECKED`. `DEFERRED` appears ONLY for step 5 and the
  sign-off block: 1a, 1b, 2, 3, 4a, 4b, 4c, 5 (FAIL / NOT CHECKED / DEFERRED only; no PASS), 6, 7 (PASS only per the step 7 owner-decision rule: Running from arming, reading at least 30 s after arming), 8.1, 8.2, 8.3,
  8b (items 1 to 6, status OPTIONAL), 9.
- NOT CHECKED list (every step or sub-step not run, and why): ____

---

## Sign-off

> This block is intentionally EMPTY. Only the human fills it in. An agent must not fill it in, tick it,
> or copy text into it.

Name: 

Date: 

Result:  [ ] PASS     [ ] DEFERRED

Explicit deferral: if DEFERRED, list the steps not run and why, on the next lines.




---

## Where results go

- Post the filled template as a comment on `pluto-atom-4/freecad-workspace#338`.
- `pluto-atom-4/freecad-workspace#352` closes only by the human.
- `pluto-atom-4/balancing-robot-controller#34` closes only by the human, after sign-off.
- Until sign-off, no hardware validation is claimed anywhere (`pluto-atom-4/freecad-workspace#352` and
  `pluto-atom-4/freecad-workspace#338` status of 2026-10-06).

---

## Open decisions for the owner

As of 2026-10-27 every item (a) to (g) is DECIDED; none is open.

- (a) **Kill-switch order.** `pluto-atom-4/freecad-workspace#352` says test the kill switch first.
  `pluto-atom-4/balancing-robot-controller#34` puts the full set at step 8. This file does the minimal
  serial `x` in step 1b and the full set in step 8. DECIDED 2026-10-27: keep the split (minimal serial `x` kill check in step 1b before any lean test; the full set in step 8).
- (b) **Bus unplug in step 8.** `pluto-atom-4/balancing-robot-controller#34` says "unplug servo bus".
  `pluto-atom-4/balancing-robot-controller#148` has no design chosen (see G8). DECIDED 2026-10-27: switch
  the servo supply OFF instead of unplugging the bus cable (step 8.3); no hot-unplug of a powered servo cable. The `pluto-atom-4/balancing-robot-controller#148` warning is respected.
- (c) **XL330 limit and model values differ across the issues.** `pluto-atom-4/balancing-robot-controller#34` body: Velocity Limit 1023, 0.229 rpm
  per raw count (both UNVERIFIED). `pluto-atom-4/balancing-robot-controller#44` comment: Velocity Limit 1620,
  model 1190, cited to the e-Manual. The repo's `lib/dxl_units/dxl_units.h:32` has 1620, and
  `programs/c3_facts/facts_io.h` names 1190 (the owner confirmed 1190 from the XL330-M077 e-Manual on
  2026-10-27, page not recorded; the code comment is being corrected in
  pluto-atom-4/balancing-robot-controller#206). `pluto-atom-4/balancing-robot-controller#84`
  says model 1200 (the sim). Step 4 asserts none of these. DECIDED 2026-10-27: read the actual values from the device and the e-Manual
  during step 4 and record them; this checklist asserts none of the numbers quoted in pluto-atom-4/balancing-robot-controller#34, pluto-atom-4/balancing-robot-controller#44 or pluto-atom-4/balancing-robot-controller#84.
- (d) **2,000,000 baud needs an EEPROM write.** Testing it needs a manual Dynamixel Wizard change. Out of
  scope for agents. (c3_facts does send 2,000,000-baud traffic at boot, step 4a, but the servos are not set to 2,000,000.)
  DECIDED 2026-10-27: stay at 57600 for the whole HIL; 2,000,000 baud is a separate later item. No EEPROM baud change is part of this sign-off; agents and probes still never do it.
- (e) **Firmware boot-time EEPROM write.** The balance firmware writes Operating Mode (register 11) at bring-up,
  up to two times (one per servo ID), each only when that servo is not already in velocity mode
  [programs/balance/dxl_wheels.h:81-87]. It is not logged. DECIDED 2026-10-27: accepted by running step 2. G5 still forbids agent and probe EEPROM writes.
- (f) **Is step 8b required for sign-off?** It was requested by the owner's answer to
  `pluto-atom-4/balancing-robot-controller#148` Q3 and is marked OPTIONAL. DECIDED 2026-10-27: optional, not required for sign-off; record PASS / FAIL / NOT CHECKED if it is run.
- (g) **Step 7 rule: DECIDED 2026-10-27 (see step 7).** Pre-arm lean-gate ticks are allowed in the `[loop]`
  statistics; the dilution (about 1.7 percent) is recorded in the result.

---

## Sources

- `pluto-atom-4/balancing-robot-controller#34` (body and comments of 2026-10-05, 2026-10-06, 2026-10-07,
  2026-10-21), `pluto-atom-4/balancing-robot-controller#43` (`c3_probe`), `pluto-atom-4/balancing-robot-controller#44`
  (`c3_facts`, comments on baud, ramp and mode write), `pluto-atom-4/balancing-robot-controller#84`
  (hardware parity, monitor and `tee` pattern), `pluto-atom-4/balancing-robot-controller#148` (kill-switch
  options), `pluto-atom-4/balancing-robot-controller#159` (USB port and bridge notes).
- `pluto-atom-4/freecad-workspace#338` (Stage F status), `pluto-atom-4/freecad-workspace#345` (sim jitter
  tolerance; the 21500 us mean limit as cited in the 2026-10-07 comment of `pluto-atom-4/balancing-robot-controller#34`;
  its F.6 docstring is `inverted-pendulum-project/07_Simulation/hal/jitter_budget.py`), `pluto-atom-4/freecad-workspace#352`
  (replaced F.14 item), `pluto-atom-4/freecad-workspace#359` (PID sign).
- Repo files at BRC commit 17436fe: `programs/balance/config.h`, `main.cpp`, `imu_mpu6050.h`, `dxl_wheels.h`,
  `lib/wheel_servo/wheel_servo.h`, `lib/balance_supervisor/balance_supervisor.h`, `lib/dxl_units/dxl_units.h`,
  `lib/balance_core/balance_gains_generated.h`, `lib/ramp_guard/ramp_guard.h`, `programs/c3_facts/main.cpp`,
  `programs/c3_facts/facts_io.h`, `wokwi/chips/wokwi-ttl-servo.c`, `CLAUDE.md`.
