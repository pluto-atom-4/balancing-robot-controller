# Hardware-in-the-loop checklist: `balance-xiao_c3`

> **Status: DRAFT. NOT VALIDATED.** Nothing in this file is validated until a human runs it on the
> real board and fills in the boxes. Agents never flash, run, probe, or tick a box, and never fill
> the sign-off block. Drafted 2026-10-27 from the issues below. This draft was not run on hardware.

Tracking: `pluto-atom-4/balancing-robot-controller#34` (this checklist, part of
`pluto-atom-4/balancing-robot-controller#22`). It replaces `pluto-atom-4/freecad-workspace#352`
(F.14). Results go to the Stage F parent `pluto-atom-4/freecad-workspace#338` (see the end of
this file).

## Purpose and scope

This is the manual checklist a human follows to run the balance firmware on a real Seeed XIAO
ESP32-C3 with the real MPU-6050 and real Dynamixel XL330 wheel servos. It checks the things the
simulation cannot: the servo bus, the IMU, the gyro bias, the real sign conventions, the real loop
timing, and the kill paths.

- In scope: the `balance-xiao_c3` env (LQR controller) and the `balance-xiao_c3-pid` env (PID
  controller), both from `platformio.ini`. The XL330 servos on the bus through the FE-URT-1 adapter
  on `Serial1` (GPIO20 RX / GPIO21 TX). The MPU-6050 on I2C (SDA = D4, SCL = D5).
- Not in scope: the Feetech STS3032 (planned for the final robot, untested, not part of this run);
  the final robot power path (`pluto-atom-4/balancing-robot-controller#178`, deferred); Webots;
  the ESP32-S3 (a future enhancement).

**Honesty rule.** Every constant, unit and baud in this file is either read from the repo (with
the file and line in brackets) or marked UNVERIFIED. A step only becomes PASS when the human has
seen the observation on the real hardware and written it down. "Not checked" and "deferred" are
valid results. No step in this file says that the baud, the servo IDs, the velocity units, the
velocity limit, the PID sign, or the STS3032 is verified.

How to read the boxes: each step has `[ ] PASS`, `[ ] FAIL`, and `[ ] NOT CHECKED`. Tick exactly
one, and write the notes under it. A FAIL is a result too: record it, do not work around it.

## Safety gates (read before any step)

All of these must be true before the servo bus is powered in step 2. Record each one in step 1.

- **G1. Wheels OFF the ground.** The robot stands on a stand with the wheels free. Nothing may
  touch the wheels or the horns.
- **G2. Servo supply is separate and current-limited.** The servo power comes from its own
  supply, current-limited, with a common GND between that supply, the FE-URT-1 adapter and the
  XIAO. [pluto-atom-4/balancing-robot-controller#34 step 1]
- **G3. Clear area.** Nobody's hands are near the wheels or the horns while the servos are powered.
- **G4. Kill check before any motion.** The minimal serial `x` kill check in step 1b must pass
  before any lean test in steps 5 and 6.
- **G5. No EEPROM writes by agents or probes.** No baud, ID, Operating Mode or Velocity Limit
  write is done by an agent or by any probe in this file. Any Dynamixel Wizard change is a manual
  human action, listed in the steps only as a human action. [pluto-atom-4/balancing-robot-controller#44]
- **G6. No autonomous hardware action.** Every flash, run, monitor and lean in this file is done by
  the human, with the command shown.
- **G7. One USB port user at a time.** Only one process may hold the USB port. Close
  `pio device monitor` before another tool opens the port. [pluto-atom-4/balancing-robot-controller#159]
- **G8. Do not hot-unplug a powered servo bus.** The servo supply is switched off first. See open
  decision (b). [pluto-atom-4/balancing-robot-controller#148]
- **G9. Do not disable a firmware kill path** to get a test through. Changing `programs/balance/config.h`
  or a kill path needs a design issue, not this checklist. [CLAUDE.md, "Balance safety design"]

Key constants used below (all in `programs/balance/config.h` unless stated; all UNVERIFIED on
hardware):

- Servo baud `kServoBaud` = 57600 (the only baud the firmware uses). `kServoBaudRequested` = 2000000
  (untested) [config.h:11-14].
- Servo IDs `kServoIdLeft` = 1 and `kServoIdRight` = 2 (ID 2 unconfirmed) [config.h:15-16].
- Wheel signs `kLeftWheelSign` = +1 and `kRightWheelSign` = -1 (unconfirmed) [config.h:17-18].
- Wheel speed cap `kMaxWheelRadS` = 1.0 rad/s [config.h:22].
- Servo write failures before latch `kServoFailMax` = 3 [config.h:23].
- Loop nominal `kLoopUs` = 20000 us (50 Hz) [config.h:29].
- Jitter "over" threshold `kJitterBudgetUs` = 29000 us [config.h:34].
- Tilt cutoff `kTiltCutoffRad` = 0.5 rad [config.h:38].
- Startup lean gate `kStartupLeanGateRad` = 0.1 rad, held for `kLeanGateHoldTicks` = 25 ticks
  [config.h:41-42].
- IMU read failures before latch `kImuFailMax` = 3 [config.h:43].
- Output scale `kWheelCmdScale` = 1.0 [config.h:46]; wheel writes enabled `kWheelWritesEnabled` = true
  [config.h:48].

---

## Step 1. Safety setup and minimal kill check

**Precondition.** The gates G1 to G9 are read. The robot is on its stand.

**Action 1a (physical).** Check and record each gate: G1 wheels off the ground, G2 servo supply
separate and current-limited with common GND, G3 clear area. Set the servo supply limit to the value
the owner has chosen (the value is not in this repo; record the number you used).

**Action 1b (minimal serial kill check).** This needs the firmware from step 2, so run it as the
first action in the step 2 session, after the boot capture is saved and before any lean (steps 5
and 6). Type `x` in the serial monitor.

**Expected 1b.** The log shows a line `[safe] LATCHED cause=...` (the cause text is printed by the
firmware), then `[safe] torque off confirmed`. The latch is terminal: reset the board to re-arm
(the firmware says "reset to re-arm"; there is no re-arm command) [programs/balance/main.cpp:123,
139]. If the log shows `[safe] TORQUE OFF NOT CONFIRMED`, power the servo supply OFF at once, and
record it.

**Record.**
- Gates G1 to G3: `[ ] PASS  [ ] FAIL  [ ] NOT CHECKED` Notes: ____
- 1b minimal serial `x` kill: `[ ] PASS  [ ] FAIL  [ ] NOT CHECKED` Notes (paste the latch line and the torque line): ____

**Unverified.** The cause text and the torque-off behaviour on this hardware. Whether a 3-second
wait is needed before the kill is a human judgement.

---

## Step 2. Flash the firmware and capture the boot log

**Precondition.** Step 1a PASS. Wheels off the ground. Servo supply ON, current-limited. Robot still.

**Action.**
```bash
cd ~/Documents/platform-io-workspace/balancing-robot-controller
pio run -e balance-xiao_c3 -t upload
pio device monitor -e balance-xiao_c3 | tee balance_boot.log
```
Reset the board and keep the robot still for the whole boot. The upload form and the monitor form
(115200 baud) are from [CLAUDE.md, "Development Commands"]; the `| tee` pattern is from
[pluto-atom-4/balancing-robot-controller#84]. Close the monitor when the boot is captured, then do
step 1b in a new monitor session, then reset the board.

**Expected observation** (the firmware prints these lines; exact text from `programs/balance/main.cpp`):
- `[bal] boot LQR (control loop, UNVERIFIED on hardware)`
- `[wheel] <family> ready baud=57600 ids=1/2 torque on, goal 0 (UNVERIFIED on hardware)`, where
  `<family>` is the servo family name the firmware prints. The baud printed must be 57600.
  [main.cpp:79-81]
- `[imu] ready`
- `[imu] calibrating gyro bias, keep robot still`
- `[imu] bias ok y=<value> rad/s (UNVERIFIED on hardware)`, or `[imu] bias FAILED y=... std=...`
  [main.cpp:90-95]. The firmware prints the Y axis only.
- `[bal] LQR waiting for lean gate (|pitch-center|<=0.100 rad)` [main.cpp:109-110]

**Servo ping, both IDs.** The bring-up pings ID 1 and then ID 2, and latches if either fails
[lib/wheel_servo/wheel_servo.h:77-80]. The firmware does not print each ID's answer. So:
- A `[wheel] ... ready ... ids=1/2` line means both IDs answered.
- `[wheel] startup FAILED fault=1, ...` means a ping failed (fault 1 is `NoPing` in the enum order
  at [lib/wheel_servo/wheel_servo.h:15]). It does not say which ID. Record it as a result.
- A missing ID 2 is a RESULT to record. Do not change `config.h` to work around it.

**Record.** Paste the boot lines. Ping ID 1: answered / not answered / unknown. Ping ID 2:
answered / not answered / unknown. Gyro bias y value: ____ rad/s.
- `[ ] PASS` only if the `[wheel] ready` line shows `ids=1/2`, and `[imu] ready` and `[imu] bias ok` both appear.
- `[ ] FAIL` otherwise (write which line is missing or which fault was printed).
- `[ ] NOT CHECKED` if the boot was not captured.

**Unverified.** ID 1 and ID 2 (`config.h:15-16`: "TODO(human)"). The `ids=1/2` wording only proves
both pings returned; it does not prove the IDs match the physical servos. The servo family name
printed by the firmware is taken from `servoBus.family()`; the repo does not list its exact text.

---

## Step 3. Gyro bias, twice, with a power cycle

**Precondition.** Step 2 captured. Robot still and flat on the stand. Wheels still off the ground.

**Action.**
1. Keep the robot still for the whole calibration. [pluto-atom-4/balancing-robot-controller#34 step 3
   says "still 5 s"; the firmware takes 500 samples in `calibrateGyroBias`, `imu_mpu6050.h:48`. The
   sample rate that makes 500 samples take 5 s is not stated in the repo: UNVERIFIED.] Record the
   `[imu] bias ok y=` value (value 1).
2. Power cycle the board with the robot still. Capture the boot again with the same command as step 2,
   into `balance_boot_2.log`. Record the second `[imu] bias ok y=` value (value 2).
3. Record the difference between value 1 and value 2.

**Expected.** Both runs print `[imu] bias ok`. The two values agree. The repo has no numeric limit
for "agree": the human judges it and writes the difference down. The firmware's limits are
`kMaxBiasRadS` = 0.15 rad/s per axis and `kMaxBiasVar` = 4.0e-4 (rad/s)^2, both marked "UNVERIFIED
guesses" [imu_mpu6050.h:26-28].

**Record.** Value 1: ____ rad/s. Value 2: ____ rad/s. Difference: ____.
- `[ ] PASS` both runs print `[imu] bias ok` and the human judges the values consistent.
- `[ ] FAIL` either run prints `[imu] bias FAILED`, or the values are not consistent (write why).
- `[ ] NOT CHECKED`.

**Unverified.** The sample rate (5 s is from the issue, not from code), the per-axis limits, and
what "consistent" means (no number in the repo).

---

## Step 4. Wheel servo facts (model, IDs, baud, velocity unit and limit, left/right sign)

This step reads facts from the device. It commands motion only in the optional sub-step 4c, with
the wheels off the ground.

**Precondition.** Step 3 done. Wheels off the ground. Servo held down or secured during 4a and 4b
(the probe is designed for a servo off the robot and held down [pluto-atom-4/balancing-robot-controller#44]).

**4a. Read the device (no motion).** Use the existing bench probe `c3_facts`:
```bash
pio run -e c3_facts-xiao_c3 -t upload
pio device monitor -e c3_facts-xiao_c3 | tee c3facts.log
```
Its output includes the keys `servo1_ping` and `servo2_ping` (separate booleans; a missing ID is
reported and is not fatal) [programs/c3_facts/main.cpp:234-237], the servo model and name, the
Velocity Limit (device-read versus assumed), the Operating Mode, the torque state and the present
velocity at rest [CLAUDE.md, "Diagnostic / probe programs"]. The probe turns torque OFF at boot and
on every exit.

After 4a and 4b, flash the balance firmware again before step 5:
```bash
pio run -e balance-xiao_c3 -t upload
```

**4b. Baud actually used.** Record the baud the servos answered at. The firmware is set to 57600
(`kServoBaud`, `config.h:11`, proven in `tilt_servo`). The 2,000,000 baud is requested by the owner,
UNTESTED on C3 with FE-URT-1, and needs a manual Dynamixel Wizard change that writes EEPROM. That
change is NOT done by this checklist (see open decision (d)). Record only what answered:
`baud_used = ____`.

**4c. Velocity unit and limit, and left/right sign (optional, commands motion; human opt-in).**
- Velocity unit: the repo says 0.229 rev/min per raw count [lib/dxl_units/dxl_units.h:10]. The
  header marks this as matching the XL330-M077 e-Manual (`https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/`),
  but it does not give a page number. Read the page on the e-Manual, write its page or section in
  the record, and compare: UNVERIFIED until you do.
- Command a known rad/s: the only repo tool that commands motion is the ramp in `c3_facts`. Build it
  with `PLATFORMIO_BUILD_FLAGS="-D C3FACTS_RAMP" pio run -e c3_facts-xiao_c3 -t upload`
  [pluto-atom-4/balancing-robot-controller#44 comment]. The ramp is positive only, capped at 20 percent
  of the Velocity Limit read from the device, about 15 s, 20 s hard timeout. It needs a typed `GO` and
  Enter within 60 s, then a 5 s countdown; any key aborts. Read Present Velocity from the
  `ramp_row` lines (`C3FACTS ramp_row <t_ms>,<goal>,<present> ms_raw_raw`).
- The ramp drives ID 1 only [programs/c3_facts/main.cpp:332]. So the ramp checks ID 1 only.
  **Right-wheel (ID 2) velocity and the left/right sign are NOT CHECKED by this probe.** The
  human may check the sign by hand (see record); no repo tool does it.
- Velocity Limit: the device value is read by `c3_facts`. The repo's constant is 1620
  [lib/dxl_units/dxl_units.h:32]. Do not assert either value here (open decision (c)).

**Expected.**
- 4a: the model name is printed, `servo1_ping` is true, and `servo2_ping` is whatever the device
  says (record it; false is a result).
- 4c: Present Velocity follows the sign of the positive goal. The ramp rows show the goal and the
  present velocity. Whether the raw-to-rad/s conversion matches the e-Manual is for the human to record.

**Record.**
- 4a model / name: ____. Operating Mode: ____. Torque state: ____. Velocity Limit (device): ____.
  `servo1_ping`: ____. `servo2_ping`: ____. Present velocity at rest: ____.
  `[ ] PASS  [ ] FAIL  [ ] NOT CHECKED`
- 4b baud used: ____ (`[ ] PASS` only if the baud that answered is recorded; `[ ] NOT CHECKED` for 2,000,000).
- 4c velocity unit vs e-Manual page ____ (page number required): `[ ] PASS  [ ] FAIL  [ ] NOT CHECKED`
- 4c left/right sign (ID 1 by ramp; ID 2 by hand, `[ ] NOT CHECKED` unless a human checked it): ____

**Unverified.** Servo model (repo has 1200 in the sim and 1190 in `dxl_units` per the issue comments;
the device value is what counts). IDs (see step 2). The velocity unit (0.229) and the Velocity
Limit (open decision (c)). The left/right sign (`config.h:17-18`). The right-wheel velocity.

---

## Step 5. PID sign check (optional but recommended)

**Precondition.** Step 4 done, and the balance firmware is flashed again or replaced by the PID env.
Step 1b PASS. Wheels off the ground. Robot upright on the stand.

**Action.**
```bash
pio run -e balance-xiao_c3-pid -t upload
pio device monitor -e balance-xiao_c3-pid | tee pid_lean.log
```
The build command for this env is in the repo; the `-t upload` form is the same as the LQR env
[CLAUDE.md, "Development Commands"]. The PID env prints `[bal] PID ...` lines. Hold the robot upright
until the firmware prints `[bal] PID ARMED (UNVERIFIED sign, wheels off ground)`. Then tilt the
robot slightly forward by hand, note which way the wheels turn, return upright, and tilt slightly
back. Keep the tilt small: the firmware latches above `kTiltCutoffRad` = 0.5 rad [config.h:38].

**Expected.** The wheels turn toward the lean (the expectation written in
[pluto-atom-4/balancing-robot-controller#34] step 5). The PID sign is UNVERIFIED in simulation too
([pluto-atom-4/freecad-workspace#359]), and PID stability is untested. A sign that is opposite to the
expectation is a FAIL result, not a reason to change code here.

**Record.** Forward lean: wheels turned ____. Back lean: wheels turned ____.
- `[ ] PASS` the wheel direction matches the expectation.
- `[ ] FAIL` it is opposite (write which way).
- `[ ] NOT CHECKED`.
- `[ ] DEFERRED` (reason: ____). Deferral is a valid result here.

**Unverified.** The PID sign, the PID gains (`kPidKp` = 1.0, `kPidKi` = 0.1, `kPidKd` = 0.05,
`lib/balance_core/balance_gains_generated.h:12`), and PID stability.

---

## Step 6. LQR sanity check

**Precondition.** Step 1b PASS. Wheels off the ground. Robot upright on the stand.

**Action.**
```bash
pio run -e balance-xiao_c3 -t upload
pio device monitor -e balance-xiao_c3 | tee lqr_lean.log
```
Hold the robot upright until `[bal] LQR ARMED (UNVERIFIED sign, wheels off ground)` appears. Then,
with the robot still, record the `[bal] pitch=... cmd=... state=... cause=... imu=...` lines
(printed at 10 Hz [main.cpp:184-189]). Then lean slowly by small steps, forward and back, and record
`pitch` and `cmd` at each step.

**Expected.**
- The sign of `cmd` follows the lean.
- The size of `cmd` grows with the tilt.
- The robot settles near an equilibrium pitch. Record that pitch from the `pitch` value at rest. The
  simulation value `kThetaRefRad` = -0.1086 rad [lib/balance_core/balance_gains_generated.h:10] is a
  SIM plant value. Do not assume the real equilibrium equals it: compare and record the difference.
- Gyro: the `[bal]` line does NOT print the gyro rate, so "command proportional to the gyro" cannot
  be checked from this log. It stays NOT CHECKED unless a human adds a print (no code change in this
  checklist).

**Record.** Pitch at rest: ____ rad. Sign agrees with lean: yes / no. Equilibrium vs -0.1086: ____.
- `[ ] PASS` command sign and size follow the tilt, and the equilibrium is recorded.
- `[ ] FAIL` the sign or size is wrong (write which).
- `[ ] NOT CHECKED`.

**Unverified.** The LQR sign, the gains (`kLqrK` values in the generated header), the equilibrium,
the gyro-proportionality claim (not logged).

---

## Step 7. Loop timing

**Precondition.** Step 3 done (gyro calibrated). The robot is upright in the lean gate. Wheels off the
ground. Record how many servos are connected (one or two). Writes to the servos happen only in the
Running state, so the timing depends on the servo count [pluto-atom-4/balancing-robot-controller#34,
comment of 2026-10-07].

**Action.** Run the balance env (step 6 session is fine). Leave it running about 30 s after the
`[bal] ... waiting for lean gate` line. Then read the `[loop]` line nearest 30 s.
- The `[loop]` line prints once per second (`main.cpp:44`, `190-197`). Take the line about 30 lines
  after the lean-gate message.
- The statistics are reset when the gyro calibration finishes (`main.cpp:101-104`), so the window
  starts at the lean-gate message.
- Line format: `[loop] min=<us> mean=<us> max=<us> over=<n> n=<n> us`.

**Pass criteria** (exactly as written in the 2026-10-07 comment of
[pluto-atom-4/balancing-robot-controller#34]):
1. `mean` at most 21500 us. This is the sustained-mean limit from
   [pluto-atom-4/freecad-workspace#345] (the sim-model tolerance, not a C3 measurement).
2. `over` = 0. `over` counts single periods above `kJitterBudgetUs` = 29000 us (`config.h:34`).
3. Sanity only, not a gate: `n` about 1500 after about 30 s at 50 Hz.

**Two different checks.** 21500 us (mean limit) and 29000 us (`kJitterBudgetUs`, the per-period
"over" threshold) are DIFFERENT checks. Passing one does not pass the other. A loop that runs
steadily at 22 ms gives over = 0 and still fails the mean check. The 29000 us figure is 20 ms
nominal plus 9 ms jitter, and the repo marks that derivation TODO(human) to confirm
[config.h:32-33].

**Record.** Servos connected: 1 / 2. Latched during the run: yes / no.
min ____ us, mean ____ us, max ____ us, over ____, n ____.
- `[ ] PASS` mean at most 21500 AND over = 0.
- `[ ] FAIL` either fails (write which).
- `[ ] NOT CHECKED`.

**Notes.** `max` can include a blocking torque-off retry (about 240 ms, per the 2026-10-07 comment)
after a latch. Do not read a latched run as a timing result. The real C3 numbers come from the
`c3_probe` run in [pluto-atom-4/balancing-robot-controller#43], not yet run; compare when it exists.

---

## Step 8. Full kill-switch set

Each sub-check gets its own box. After every latch, reset the board to re-arm. Keep the servo
supply current-limited.

**8.1 Tilt cutoff.** Precondition: the robot is armed (`ARMED` printed). Action: tilt the robot past
0.5 rad from centre (`kTiltCutoffRad`, `config.h:38`), by hand, slowly. Expected: a
`[safe] LATCHED cause=...` line, then `[safe] torque off confirmed`. Record the cause text and the
torque line. `[ ] PASS  [ ] FAIL  [ ] NOT CHECKED`

**8.2 Serial `x` (full, while Running).** Precondition: armed. Action: type `x`. Expected: the same
latch and torque-off lines as 1b, with the wheels stopped by eye. `[ ] PASS  [ ] FAIL  [ ] NOT CHECKED`

**8.3 IMU unplug.** Precondition: armed, robot on the stand. Action: disconnect the MPU-6050 (I2C
SDA = D4, SCL = D5) while the firmware runs. Expected: after `kImuFailMax` = 3 consecutive read
failures [config.h:43], a latch with a `[safe] LATCHED` line and torque off confirmed. Record the
cause text. `[ ] PASS  [ ] FAIL  [ ] NOT CHECKED`

**8.4 Servo failsafe.** Precondition: armed. Action: see open decision (b). The suggested method is to
switch the servo supply OFF (not a hot-unplug of a powered bus, G8). Expected: after `kServoFailMax` =
3 consecutive write failures [config.h:23], a latch. The firmware then tries torque off; it may print
`[safe] TORQUE OFF NOT CONFIRMED, retrying` or `... gave up: REMOVE SERVO POWER` (main.cpp:128-131).
Record every line. `[ ] PASS  [ ] FAIL  [ ] NOT CHECKED`

**Unverified.** Every cause name and every timing above on this hardware. The servo-off method
(open decision (b)). The results of 8.3 and 8.4 depend on the IMU and bus wiring as built.

---

## Step 8b. OPTIONAL: XL330 bus watchdog (register 98)

**Optional.** This item is NOT required for sign-off unless the owner adds it to the sign-off
block. It comes from the 2026-10-21 comment of [pluto-atom-4/balancing-robot-controller#34], posted
for the owner's answer to [pluto-atom-4/balancing-robot-controller#148] Q3. It is not run.

**Source.** The XL330-M077 e-Manual, register address 98: 1 byte, RAM, range 1 to 127, unit 20 ms,
0 = off. If no valid instruction arrives within the period while Torque Enable is 1, the servo
"stops", the register reads -1, and goal writes return a Data Range Error until 0 is written to
address 98. The manual page does not say that torque is disabled on timeout. UNVERIFIED for this
servo, and for the sim model (which reports model 1200). Part of the e-Manual page was not read.

**Precondition.** Servo off the robot and held down (as in step 4). Separate supply and common GND,
current-limited supply. A tool that can write and read single registers (a Dynamixel Wizard 2.0
session, or a throwaway bench sketch). NO balance firmware and NO control loop running.

**Items** (record each with a PASS / FAIL / NOT CHECKED box):
1. Register 98 reads 0 (off) at power-up.
2. Write a non-zero value (record the value and the unit used; nominal 20 ms per count), enable
   torque, command a small goal velocity, then send nothing. Record whether the servo stops, how long
   after the last instruction, and the time against the nominal period.
3. After the timeout: read back Torque Enable, Present Velocity, register 98 (expected -1 per the
   manual) and the error returned by a goal write.
4. Clear it by writing 0 to register 98. Record what else is needed to recover (torque off and on,
   or a reboot).
5. Refresh test: with a value set, send a valid instruction faster than the period for 30 s. Record
   that the servo does not stop.
6. Which instructions reset the timer (a read, a write, a ping). Record it, because the control loop
   would use that one.

**Result rule.** A pass here does NOT make the watchdog a safety layer by itself. The owner decides
how it is used (`pluto-atom-4/balancing-robot-controller#148` option D, with the software kill
paths kept). Do not change any firmware kill path or any `config.h` constant on the strength of
this test; that needs a design issue.

---

## Step 9. OPTIONAL: supervised free-balance attempt

**Optional.** Do this only after steps 1 to 8 are PASS, or DEFERRED with a written reason.

**Precondition.** A written kill plan: serial `x`, tilt cutoff, and the servo supply switched OFF as
the last resort. A human present for the whole run. The robot's wheels free to move on a surface
the human has chosen.

**Action.** The human runs the balance env as the human decides. This checklist sets no run length,
no speed and no pass threshold for a free balance. No repo number says what a successful free
balance is.

**Record.** `[ ] PASS  [ ] FAIL  [ ] NOT CHECKED`. Notes: ____

---

## Results template

Fill this in and post it as the results comment (see "Where results go"). Do not fill it in the
repo file.

- Date: ____
- Board: Seeed XIAO ESP32-C3 (serial number or label: ____)
- Firmware: env `balance-xiao_c3` / `balance-xiao_c3-pid` (circle one); build flags: ____
- Firmware SHA (BRC commit): ____
- Gains hash (`kGainsHash` in `lib/balance_core/balance_gains_generated.h` as built; the value at
  BRC commit 17436fe is 0x1e60b0fc): ____
- Servos connected: 1 / 2. Baud used: ____ (57600 proven; 2,000,000 untested).
- Servo supply limit used: ____
- Per-step observation: steps 1 to 9, each with PASS / FAIL / NOT CHECKED / DEFERRED and notes.
- NOT CHECKED list (write every step or sub-step not run, and why): ____

---

## Sign-off

> This block is intentionally EMPTY. Only the human fills it in. An agent must not fill it in,
> tick it, or copy text into it.

Name: 

Date: 

Result:  [ ] PASS     [ ] DEFERRED

Explicit deferral: if DEFERRED, list the steps not run and why, on the next lines.




---

## Where results go

- Post the filled template as a comment on `pluto-atom-4/freecad-workspace#338`.
- `pluto-atom-4/freecad-workspace#352` closes only by the human.
- `pluto-atom-4/balancing-robot-controller#34` is closed only by the human, after sign-off.
- Until sign-off, no hardware validation is claimed anywhere (`pluto-atom-4/freecad-workspace#352`
  status of 2026-10-06 and `pluto-atom-4/freecad-workspace#338`).

---

## Open decisions for the owner (not decided here)

Each item is a question. Nothing here is decided by this file.

- (a) **Kill-switch order.** `pluto-atom-4/freecad-workspace#352` says to test the kill switch
  first. `pluto-atom-4/balancing-robot-controller#34` puts the full kill set at step 8. This file
  does a minimal serial `x` check in step 1b and the full set in step 8.
  TODO(human): confirm this order, or move the full set earlier.
- (b) **Bus unplug in step 8.** `pluto-atom-4/balancing-robot-controller#34` says "unplug servo bus".
  `pluto-atom-4/balancing-robot-controller#148` warns against hot-unplugging a powered servo. This
  file suggests switching the servo supply OFF instead (step 8.4).
  TODO(human): decide the method.
- (c) **XL330 limit and model values differ across the issues.** `pluto-atom-4/balancing-robot-controller#34`
  body: Velocity Limit 1023, 0.229 rpm per raw count (both UNVERIFIED). `pluto-atom-4/balancing-robot-controller#44`
  comment: Velocity Limit 1620 and model 1190, cited to the e-Manual. `pluto-atom-4/balancing-robot-controller#84`:
  model 1200 (sim). The repo's `lib/dxl_units/dxl_units.h:32` has 1620. Step 4 does not assert any of
  these; it asks for the device values and the e-Manual page.
  TODO(human): choose which values are accepted after reading the device and the e-Manual.
- (d) **2,000,000 baud needs an EEPROM write.** The servos are at 57600 (factory). Testing 2,000,000
  needs a manual Dynamixel Wizard change that writes EEPROM. This is out of scope for agents.
  TODO(human): decide whether and when a human runs a 2,000,000 test.

---

## Sources

- `pluto-atom-4/balancing-robot-controller#34` (body and comments of 2026-10-05, 2026-10-06,
  2026-10-07, 2026-10-21).
- `pluto-atom-4/balancing-robot-controller#43` (`c3_probe`), `pluto-atom-4/balancing-robot-controller#44`
  (`c3_facts`, its comments for the baud and the ramp), `pluto-atom-4/balancing-robot-controller#84`
  (hardware parity, monitor and `tee` pattern), `pluto-atom-4/balancing-robot-controller#148`
  (kill-switch options), `pluto-atom-4/balancing-robot-controller#159` (USB port and bridge notes).
- `pluto-atom-4/freecad-workspace#338` (Stage F status), `pluto-atom-4/freecad-workspace#345` (sim
  jitter tolerance; the 21500 us mean limit as cited in pluto-atom-4/balancing-robot-controller#34 of 2026-10-07),
  `pluto-atom-4/freecad-workspace#352` (the replaced F.14 item), `pluto-atom-4/freecad-workspace#359`
  (PID sign).
- Repo files at BRC commit 17436fe: `programs/balance/config.h`, `programs/balance/main.cpp`,
  `programs/balance/imu_mpu6050.h`, `lib/wheel_servo/wheel_servo.h`, `lib/dxl_units/dxl_units.h`,
  `lib/balance_core/balance_gains_generated.h`, `programs/c3_facts/main.cpp`, `CLAUDE.md`.
