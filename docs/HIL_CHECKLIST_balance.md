# Balance hardware-in-the-loop checklist (balance-xiao_c3)

Human gate for BRC #34 (Part of #22). Stage F hardware validation is **not claimed** by this
document or by the repo until the sign-off block at the end is filled in by a human.

- Nothing in `balance` has been run on hardware. This file is a procedure, not a result.
- Agents and probes do not run any step below. Only the human, with the board in hand.
- Every step has an expected observable and a PASS/FAIL box. If a step is not run, mark it
  `not checked` in the results comment. Do not tick PASS for something you did not see.
- Constants are in `programs/balance/config.h`. This checklist does not change any of them.

## Inputs and their status

| Input | Value | Status |
|---|---|---|
| Timing budget (freecad-workspace#345, `jitter_budget.py`) | nominal 20 ms; sustained mean budget 21.5 ms (27 ms x 0.8); up to 3 missed updates; jitter +-9 ms | Sim-model tolerance, NOT a C3 measurement |
| Firmware over-budget threshold `kJitterBudgetUs` | 29000 us | Sim-derived, UNVERIFIED on C3 |
| Sim rest pitch / LQR `THETA_REF` | -0.1086 rad | SIM plant value, UNVERIFIED on the robot |
| Servo bus | Serial1, GPIO20 (RX) / GPIO21 (TX), FE-URT-1 | Wiring per `tilt_servo` code |
| Servo baud | 57600 (the rate the existing `tilt_servo` code uses) | Bench result still to be recorded |
| Servo baud requested | 2,000,000 | UNTESTED on C3 + FE-URT-1 |
| Servo IDs | 1 and 2 | ID 1 from `tilt_servo`; ID 2 UNCONFIRMED |
| Wheel mirroring | left sign +1, right sign -1 | UNCONFIRMED |
| Servo model / velocity unit / velocity limit | see Open item (c) | UNVERIFIED, record what the device reports |
| Tilt cutoff `kTiltCutoffRad` | 0.5 rad | UNVERIFIED |
| Startup lean gate | 0.1 rad about center, held 25 ticks (0.5 s) | UNVERIFIED |
| Stall threshold `kStallUs` | 100 ms | UNVERIFIED |

## Decisions and open items

These are decisions already made for the contradictions between #34, #44, #84 and #148. They
are open items for the human to confirm, not settled facts.

- **(a) Kill-switch order.** Run a quick serial `x` smoke test early (step 1). Run the full
  kill-switch set last (step 8).
- **(b) Bus unplug.** Do not hot-unplug a powered servo bus cable (BRC #148; ROBOTIS warns
  against connecting or disconnecting cables under power). For the servo-bus failsafe test,
  cut the servo supply, or use another method the human chooses. Record exactly what was done.
- **(c) XL330 values are UNVERIFIED.** The sources disagree or are assumptions:
  - Model: the Wokwi sim reports model 1200 (XL330-M288, #84). `lib/dxl_units` and #148 cite
    the XL330-M077 e-Manual. Record the model number the device reports, and its name.
  - Velocity unit: 0.229 rpm per raw unit (`lib/dxl_units`, #27, #84). UNVERIFIED.
  - Velocity limit: 1023 assumed (#27, #44). Record the value the device reports.
  - Profile Velocity / Profile Acceleration units (#84): UNVERIFIED, not used by `balance`.
  - Record the e-Manual page cited for any unit used in the results comment.
- **(d) 2M baud is a manual EEPROM write.** Only the human may change the servo baud, by hand,
  and only after recording that 57600 works. Agents and probes must never write it. The
  firmware keeps `kServoBaud = 57600` for this checklist.
- **Open: PID sign.** Deferred (freecad-workspace#359, deferred 2026-10-27). Step 5 allows
  `DEFERRED`.
- **Open: servo ID 2 and mirroring.** Unconfirmed. Note: `WheelPair::begin()` pings both IDs
  and fails startup if either is missing. A missing ID 2 therefore stops the firmware from
  arming (`[wheel] startup FAILED`). That is a result to record. Steps 6 to 9 cannot run
  until the human resolves it. This checklist does not add a single-wheel mode.
- **Open: real equilibrium angle.** Not known. Measured in step 6.
- **Open: bus-watchdog test (step 8b)** is an owner-proposed bench test (BRC #34 comment).
  Its behaviour on the real XL330 is unknown.
- **Out of scope:** STS3032 (planned, untested, not part of this run); BOOT-button kill (a TODO,
  not implemented).

## Suggested order

1. Run any read-only monitoring or display chain first. No torque, no balance firmware.
2. Flash `balance-xiao_c3` as a separate, deliberate flash (step 2). Run the step 1 serial
   `x` smoke test right after this first boot, before the other step 2 checks.
3. Flash `balance-xiao_c3-pid` only for step 5, then return to `balance-xiao_c3` for step 6.

## Step 1: Preconditions and safety setup

Setup:
- Seeed XIAO ESP32-C3, MPU-6050 on I2C SDA=D4 (GPIO6), SCL=D5 (GPIO7).
- FE-URT-1 on Serial1: RX=D7 (GPIO20), TX=D6 (GPIO21). Dynamixel XL330 servos on the bus.
  Feetech STS3032 is not part of this run.
- Robot on a stand, wheels off the ground, nothing touching the floor.
- Servos on a separate, current-limited supply. Set the current limit before connecting and
  record its value. Common GND between the supply, the FE-URT-1 and the C3.
- Power off before connecting or disconnecting any cable.
- A way to cut servo power within reach, and a USB serial monitor at 115200.
- No EEPROM, baud, ID or limit write is done in this checklist.

Serial `x` smoke test (run right after the first balance boot, see step 2):
- Send `x` on USB serial.
- Expected: `[safe] LATCHED cause=SerialKill`, then `[safe] torque off confirmed`.
- Read back Torque Enable (address 64) on each servo: expected 0.
- Expected: no re-arm without a board reset.

- [ ] PASS  - [ ] FAIL

Notes (record supply limit, cable routing, anything unusual):

## Step 2: Flash and boot check

Command: `pio run -e balance-xiao_c3 -t upload`, then `pio device monitor -e balance-xiao_c3`.

Expected observables:
- `[bal] boot LQR` line.
- `[imu] ready`.
- `[imu] bias ok y=... rad/s (UNVERIFIED on hardware)`. If it fails, `[imu] bias FAILED` is the result.
- `[wheel] ready baud=57600 ids=1/2`, meaning both IDs answered the ping.
- If one ID does not answer: `[wheel] startup FAILED fault=...`. Record the ID that is missing
  and the fault number. This is a RESULT, not a failure of the checklist.

Record: IMU ready (yes/no); bias y value; ping result for ID 1; ping result for ID 2.

- [ ] PASS (both IDs answer, IMU ready)  - [ ] FAIL (IMU not found or other boot error)
- [ ] RESULT recorded: ID 2 missing (firmware does not arm; go to step 3 and step 4 only)

## Step 3: Gyro bias

Expected observables:
- `[imu] calibrating gyro bias, keep robot still` for about 5 s with the robot still.
- The bias value and standard deviation are printed.
- Threshold: TODO(human). No bias limit is documented in the repo. Set it before the run and
  record it here: ______ rad/s.

Repeat after a power cycle. The values should be consistent between the two runs.

Record: bias y and std (run 1); bias y and std (run 2, after power cycle); threshold used.

- [ ] PASS (|bias| under the recorded threshold, both runs consistent)  - [ ] FAIL

## Step 4: Servo facts

Source: the bench facts probe `c3_facts` (BRC #44) with servos off the robot and held down.
Record which source produced each value (c3_facts run, or a human-run equivalent).

Record each item:
- Servo model number and name the device reports. Expected: XL330 family. Compare with (c).
- IDs that answered. Expected: 1 and 2 (2 unconfirmed).
- Baud actually used. 57600: expected to work (existing code). 2,000,000: record whether it
  worked, or `not tried`. Do not change the baud here (see Decision d).
- Velocity limit read from the device, and the velocity unit used by the firmware (0.229 rpm
  per raw, UNVERIFIED). Command a known small positive speed (for example via `c3_facts`
  ramp, capped at 20 % of the device limit) and read present velocity back. Record both.
- Left/right sign: command a positive wheel speed to the balance code path (wheels off the
  ground). Expected: both wheels turn in the forward direction of the robot. If one turns the
  other way, record it. Do not change `kLeftWheelSign` / `kRightWheelSign` here.

Checks:
- [ ] PASS: model, IDs, baud used and velocity limit recorded with the device-read values
- [ ] PASS: commanded speed and read-back speed agree within the tolerance the human sets (record it)
- [ ] PASS: left/right direction matches the expected sign, or the mismatch is recorded
- [ ] FAIL / other (record which)

## Step 5: PID sign check

Flash `balance-xiao_c3-pid` (`-D BALANCE_CONTROLLER_PID`). Note: the PID sign is UNVERIFIED in
the sim too (freecad-workspace#359).

Procedure: hold the robot level until the firmware reports `[bal] ... ARMED`. Then lean the
robot forward by hand, gently and slowly, staying well under the 0.5 rad cutoff. Watch the
wheels.

Expected observable: the wheels drive toward the lean (forward lean, wheels turn forward).
Record the direction actually observed. The firmware does not decide this; the human does.

- [ ] PASS (wheels drive toward the lean; direction recorded)
- [ ] FAIL (wheels drive away from the lean; direction recorded; do not change code here)
- [ ] DEFERRED (see the DEFERRED template below; freecad-workspace#359)

## Step 6: LQR sanity

Flash `balance-xiao_c3` (LQR, default). Same level-then-lean test as step 5.

Expected observables:
- `[bal] pitch=... cmd=... state=... cause=...` at 10 Hz.
- The command magnitude grows with tilt and with gyro rate, and the sign is the same for the
  same tilt in each direction (as the controller is designed).
- Equilibrium: with the robot balanced at rest, record the `pitch=` value. Compare it with the
  sim value -0.1086 rad (`THETA_REF`, SIM plant, UNVERIFIED on the robot).

Record: measured equilibrium pitch (rad); difference from -0.1086 rad; command vs tilt
observations.

- [ ] PASS (command grows with tilt and gyro, correct sign; equilibrium recorded)
- [ ] FAIL (record observation)

## Step 7: Loop timing

Read the `[loop]` line about 30 s after the `waiting for lean gate` message. The statistics
are cumulative since calibration, and `n` should be about 1500 at 50 Hz.

Expected observable: `[loop] min=... mean=... max=... over=... n=... us`.

PASS requires BOTH:
1. mean <= 21500 us (21.5 ms, the sustained mean budget from freecad-workspace#345), and
2. over = 0 (no single period above `kJitterBudgetUs` = 29000 us).

Record: min, mean, max, over, n; number of servos connected (1 or 2).

Cautions:
- 21.5 ms and 29000 us are budgets from the sim model, not C3 measurements. The C3 numbers
  come from `c3_probe` (BRC #43), not from this step.
- `over=0` alone is not a pass. A loop that runs steadily slow can pass `over=0` and still
  fail the mean limit.
- `max` can include a blocking torque-off retry (about 240 ms) after a latch. Do not read a
  latched run as a timing result.

- [ ] PASS (mean <= 21500 us AND over=0)  - [ ] FAIL  - [ ] not checked

## Step 8: Kill-switch set (full)

Run each item with the robot armed, wheels off the ground, and a hand ready to catch it.
Record the `[safe]` lines and the servo torque read-back (address 64) after each item.

8.1 Tilt cutoff
- Lean the robot past 0.5 rad from center (about 28.6 degrees). Keep the lean slow.
- Expected: `[safe] LATCHED cause=TiltCutoff`, `[safe] torque off confirmed`, Torque Enable 0.
- [ ] PASS  - [ ] FAIL

8.2 Serial `x` (full run, after a re-arm)
- Reset the board, arm the robot, send `x`.
- Expected: `LATCHED cause=SerialKill`, torque off confirmed, no re-arm until reset.
- [ ] PASS  - [ ] FAIL

8.3 IMU unplug (failsafe)
- Disconnect the MPU-6050 with the robot armed. Record whether the disconnect was live or
  with the board off (human choice).
- Expected: after consecutive IMU read failures, `LATCHED cause=ImuFail`, torque off confirmed.
- [ ] PASS  - [ ] FAIL

8.4 Servo-bus failsafe (no hot-unplug)
- Per Decision (b), do not hot-unplug a powered bus cable. Method chosen by the human:
  ______________________ (record exactly what was done, e.g. servo supply cut).
- Expected: write failures reach the limit (`kServoFailMax` = 3), `LATCHED cause=WheelFail`,
  and either `[safe] torque off confirmed` or `TORQUE OFF NOT CONFIRMED, gave up: REMOVE SERVO POWER`.
  Record which one.
- [ ] PASS  - [ ] FAIL  - [ ] not checked

## Step 8b: XL330 bus watchdog, register 98 (optional bench test)

Proposed in the BRC #34 comments (owner answer to BRC #148 Q3). Not run. Nothing here changes
a firmware kill path or `config.h`.

Setup: servo off the robot and held down; separate current-limited supply; a single-register
tool (Dynamixel Wizard 2.0, or a throwaway bench sketch). No balance firmware and no control
loop running.

Source: XL330-M077 e-Manual, address 98, 1 byte, RAM, range 1-127, unit 20 ms, 0 = off. The
manual does not say torque is disabled on timeout. UNVERIFIED for this servo; the sim reports
M288.

Items (record the value and the unit actually used):
1. Register 98 reads 0 at power-up.
   - [ ] PASS  - [ ] FAIL
2. Write a non-zero value, enable torque, command a small goal velocity, then send nothing.
   Record whether the servo stops, how long after the last instruction, and the time against
   the nominal period.
   - [ ] PASS  - [ ] FAIL
3. After the timeout, read back Torque Enable, Present Velocity and register 98 (manual says
   -1), and the error returned by a goal write.
   - [ ] PASS  - [ ] FAIL
4. Clear it by writing 0 to register 98. Record what else is needed to recover (torque off/on,
   reboot).
   - [ ] PASS  - [ ] FAIL
5. Refresh test: with a value set, send a valid instruction faster than the period for 30 s.
   Record that the servo does not stop.
   - [ ] PASS  - [ ] FAIL
6. Which instructions reset the timer (read, write, ping). Record it; the control loop would
   use that one.
   - [ ] PASS  - [ ] FAIL

A pass here does NOT make the watchdog a safety layer by itself. The owner decides how it is
used (BRC #148 option D), with the software kill paths kept.

## Step 9: Supervised free-balance attempt (optional)

Only after steps 5 to 8 are PASS or explicitly DEFERRED, and only if the human chooses to try.
Wheels on the ground. Kill plan written first: serial `x` in a terminal, and the servo power
cut within reach. No agent takes part.

Record: duration of balance without a latch; any `LATCHED cause=...` line; what ended the run.

- [ ] PASS (balanced for the recorded duration, no latch)  - [ ] FAIL  - [ ] not attempted

## Step 10: Record and sign

Results go in a comment on freecad-workspace#338, using the template below. The #352 tracker
body lists the same fields. Mark every item that was not run as `not checked`.

Results template (copy into a comment on freecad-workspace#338):

```
Balance HIL results (human-run). Not a validation until the sign-off below is filled.
- date:
- board:
- firmware SHA (git rev-parse HEAD):
- gains hash:
- env:
- servo model / IDs / baud used / velocity limit:
- step 1 (preconditions, x smoke test):
- step 2 (boot, IMU, IDs):
- step 3 (gyro bias, two runs):
- step 4 (servo facts, sign):
- step 5 (PID sign):
- step 6 (LQR, equilibrium pitch vs -0.1086 rad):
- step 7 (loop timing vs 21.5 ms / over=0):
- step 8 (kill-switch set):
- step 8b (watchdog, if run):
- step 9 (free balance, if run):
- NOT checked:
- sign-off or deferral (human):
```

DEFERRED template (post as a comment when a step is explicitly deferred):

```
DEFERRED: step __, reason: ______, owner: ______, date: ______.
Not validated. Reference: freecad-workspace#359 (PID sign) or the open item above.
```

### Sign-off (empty; only the human fills this in)

- Name:
- Date:
- Board:
- Firmware commit:
- Env:
- Signature:

Until this block is filled, Stage F hardware validation is NOT claimed. Issue #34 closes only
after the human signs.
