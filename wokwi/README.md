# Shared Wokwi assets

Simulation-only. Nothing here is validated on real hardware.

## Purpose

Custom Wokwi chips and the SVG-to-background converter, shared by the program
Wokwi sims (`tilt_servo`, `wokwi_ttl_servo`).

## Layout

- `chips/wokwi-fe-urt-1.c` + `.chip.json`: half-duplex FE-URT-1 adapter (pin-level repeater that suppresses echo).
- `chips/wokwi-ttl-servo.c` + `.chip.json` + `.svg`: Dynamixel XL330-style servo chip (separate RX/TX bridged by the adapter to a shared DATA bus).
- `chips/wokwi-ttl-servo.bg.h`: GENERATED baked background for the servo display. Committed. Do not hand-edit.
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
- `make wokwi_ttl_servo-sim`: needs `WOKWI_CLI_TOKEN`.

The first run of `make sim-tilt_servo-xiao_c3`, `make sim-wokwi_ttl_servo-xiao_c3` or `make sim` also builds the chips via `make wokwi-chips`. That step needs network, because `wokwi-cli` downloads the WASI-SDK and `wokwi-api.h` on first use.

## Limits

- The servo chip answers ping and acks writes. It integrates goal velocity into present position (Operating Mode 11, Torque 64, Goal Velocity 104, Present Position 132).
- It does NOT model Goal Position (reg 116) or profile registers. A tilt-driven horn in the tilt_servo sim will not follow the tilt.
- The chip hard-codes servo ID 1. The balance sim does not use these chips (it needs ID 2).
- The tilt_servo sim wiring (`esp:TX`/`esp:RX` to the adapter) passed the headless CLI diagram check and pinged the servo; the GUI look is unchecked.
- Simulation-only. Not hardware-validated.
