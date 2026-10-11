# Shared Wokwi assets

Simulation-only. Nothing here is validated on real hardware.

## Purpose

Custom Wokwi chips and the SVG-to-background converter, shared by the program
Wokwi sims (`tilt_servo`, `ttl_servo`, `wokwi_ttl_servo`).

## Layout

- `chips/wokwi-fe-urt-1.c` + `.chip.json`: half-duplex FE-URT-1 adapter (pin-level repeater that suppresses echo).
- `chips/wokwi-ttl-servo.c` + `.chip.json` + `.svg`: Dynamixel XL330-style servo chip (separate RX/TX bridged by the adapter to a shared DATA bus).
- `chips/wokwi-ttl-servo.bg.h`: GENERATED baked background for the servo display. Committed. Do not hand-edit.
- `chips/ttl_servo_slew.h`: pure position-mode slew math used by the servo chip (Goal Position toward Profile Velocity), tested natively by `test/test_ttl_servo_slew`.
- `chips/wokwi-api.h`: NOT vendored. Downloaded by `wokwi-cli` next to the source on the first `chip compile`. Gitignored. The first chip build needs network.
- `chips/*.chip.wasm`: build output, produced by `wokwi-cli chip compile`.
- `tools/svg_to_pixels.sh`: converts the SVG into `wokwi-ttl-servo.bg.h`.

This directory is outside `programs/` on purpose. `src_dir = programs` means PlatformIO would compile chip C into firmware if it lived there.

## How programs reference chips

A program's `wokwi.toml` lists the chips with paths relative to the toml file:

```toml
[[chip]]
name = 'wokwi-fe-urt-1'
binary = '../../wokwi/chips/wokwi-fe-urt-1.chip.wasm'
```

The path resolves relative to the toml directory. A missing `.wasm` fails with `chip WASM file not found`. Build the chips first.

## Make targets

- `make wokwi-chips`: build all chips (`wokwi-cli chip compile`). Alias: `make wokwi_ttl_servo-chip`.
- `make wokwi-bg`: regenerate `wokwi/chips/wokwi-ttl-servo.bg.h` from the SVG. Needs inkscape and ImageMagick.
- `make sim-tilt_servo-xiao_c3`: tilt_servo sim with the servo and FE-URT-1 chips on the Serial1 pins (GPIO20/21, `esp:RX`/`esp:TX`). Needs `WOKWI_CLI_TOKEN`. A headless CLI run showed boot, `[dxl] ping ok` and `[dxl] torque on`; the GUI display was not checked.
- `make sim-ttl_servo-xiao_c3`: ttl_servo closed-loop Segway sim; the same two chips on the Serial1 pins, velocity mode. Needs `WOKWI_CLI_TOKEN`. Headless gyro push: `WOKWI_ARGS="--scenario scenario.yaml --timeout 20000"`.
- `make wokwi_ttl_servo-sim`: needs `WOKWI_CLI_TOKEN`.

`make sim-tilt_servo-xiao_c3`, `make sim-wokwi_ttl_servo-xiao_c3` and `make sim` (with the default `SIM_ENV`) also build the chips via `make wokwi-chips` whenever a chip's `.chip.wasm` is missing or older than its sources. On a fresh clone that is the first run, and it needs network: `wokwi-cli` downloaded `wokwi-api.h` on first use when it was tested (the WASI-SDK download is also done by `wokwi-cli`, not checked here).

## Tilt injection tip

To tilt the `tilt_servo` sim without moving the board, set the part attributes `accelX` and `accelZ` (in g) on the `wokwi-mpu6050` part in the diagram. For example, 20 degrees is `accelX` -0.342 and `accelZ` 0.94. Do not commit a temporary edit.

## Limits

- The servo chip answers ping and acks writes. It integrates goal velocity into present position (Operating Mode 11, Torque 64, Goal Velocity 104, Present Position 132).
- Position mode (Operating Mode 3, torque on): the chip moves present position toward Goal Position (reg 116) at Profile Velocity (reg 112) times the same 0.229 rpm unit as goal velocity. Profile Velocity 0 jumps instantly. The goal is clamped to 0..4095, stored while torque is off and applied on torque-on, and an Operating Mode write sets goal = present. Profile Acceleration (reg 108) is stored but NOT modelled. Units of regs 108/112 are UNVERIFIED against the XL330 e-Manual. Goal Velocity is ignored in position mode.
- Position-mode evidence is headless only: with an injected 20 degree tilt, the firmware goal ramped to 2272 and the chip logged `[servo] goal=2272 pos=2272 vel=100 torque=1`. The GUI horn was not checked. The chip logs `[servo] goal=.. pos=.. vel=.. torque=..` (rate-limited, position mode only) as a sim aid.
- Velocity mode (the `wokwi_ttl_servo` demo) is unchanged.
- The chip hard-codes servo ID 1. The balance sim does not use these chips (it needs ID 2).
- The tilt_servo sim wiring (`esp:TX`/`esp:RX` to the adapter) passed the headless CLI diagram check and pinged the servo; the GUI look is unchecked.
- Simulation-only. Not hardware-validated.
