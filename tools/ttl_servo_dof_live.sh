#!/usr/bin/env bash
# tools/ttl_servo_dof_live.sh
#
# SIM-ONLY, not hardware-validated. Runs the headless Wokwi ttl_servo DOF frame stream (build with
# -D TTL_SERVO_DOF_FRAMES) and pipes wokwi-cli's stdout live into the freecad-workspace bridge
# --stdin. One-way stream: the firmware PD drives the Wokwi servo chip; the Python LQR command is
# only shown on the Webots display wheel; nothing feeds back into the physics. Never builds
# firmware. See -h for usage and exit codes.

set -u

usage() {
  cat <<'EOF'
Usage: tools/ttl_servo_dof_live.sh [-s SCENARIO.yaml] [-t TIMEOUT_MS] [-f FW_ROOT] [-h] [-- BRIDGE_ARGS...]

SIM-ONLY, not hardware-validated. Live, one-way stream: wokwi-cli runs headless on programs/ttl_servo
and its stdout is piped into the freecad-workspace bridge (poc/esp32-dof/monitor/dof_wheel_bridge.py
--stdin --core lqr).
  - Two controllers: the firmware PD (drives the Wokwi servo chip, the physics) and the Python LQR
    (its command is only shown on the Webots display wheel). No feedback into the physics or the chip.
  - Sign conventions UNVERIFIED (BRC#185, FW#359, FW#377). Not hardware-validated.
  - The bridge --stdin path and the Webots display are not run end to end by the builder.

  -s SCENARIO.yaml  scenario; relative paths are looked up in programs/ttl_servo first, then in
                    the current directory (default: scenario_dof_frames.yaml)
  -t TIMEOUT_MS     wokwi-cli timeout in ms (default: 60000)
  -f FW_ROOT        freecad-workspace repo root (default: the FW_ROOT environment variable;
                    one of the two is required)
  -h                show this help
  -- BRIDGE_ARGS    everything after -- goes to the bridge unchanged, for example:
                    -- --core pid --duration 30   (a later --core wins over the built-in --core lqr)

Before running:
  1. Build the DOF firmware (the script never builds):
     PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" pio run -e ttl_servo-xiao_c3
     Afterwards rebuild without the flag: pio run -e ttl_servo-xiao_c3 (shared .pio build dir).
     The script only checks that firmware.bin exists, not that it has the flag.
  2. make wokwi-chips   (only if the chip .wasm files are missing; needs network on first run)
  3. Start Webots (world servo_wheel_dof) first, listening on UDP 5006.
  4. WOKWI_CLI_TOKEN=... FW_ROOT=/abs/path/to/freecad-workspace tools/ttl_servo_dof_live.sh

Python: the bridge runs with python3 from PATH. The FW env is `mamba run -n esp32-dof`; you can run
the whole script under it, for example: mamba run -n esp32-dof tools/ttl_servo_dof_live.sh -f <FW>

WOKWI_CLI selects the wokwi-cli binary (default: wokwi-cli on PATH).

Exit codes. wokwi-cli's own exit codes can collide with the codes below.
  0    wokwi-cli exited 0 or 42 (its timeout, which the firmware loop always reaches) and the bridge exited 0.
       42 is also what a build without the flag gives (the banner wait never matches); the script still exits 0.
  1    usage, input or environment error (one line on stderr), or the scenario/pipeline could not start
  130  stopped by SIGINT (pipeline killed)
  143  stopped by SIGTERM (pipeline killed)
  N    the bridge's nonzero exit code N (checked first)
  N    wokwi-cli's nonzero exit code N, other than 42, when the bridge exited 0
EOF
}

# Pure-bash abspath, relative to the caller's directory.
abspath() {
  case "$1" in
    /*) printf '%s\n' "$1" ;;
    *) printf '%s/%s\n' "$PWD" "$1" ;;
  esac
}

SCEN="scenario_dof_frames.yaml"
TMO="60000"
FW="${FW_ROOT:-}"

while getopts ":s:t:f:h" opt; do
  case "$opt" in
    s) SCEN=$OPTARG ;;
    t) TMO=$OPTARG ;;
    f) FW=$OPTARG ;;
    h) usage; exit 0 ;;
    :) echo "error: -$OPTARG needs a value" >&2; usage >&2; exit 1 ;;
    *) echo "error: unknown option -$OPTARG" >&2; usage >&2; exit 1 ;;
  esac
done
# getopts stops after "--" (OPTIND-1 points at it); anything else left over is an error.
SEP=""
eval "SEP=\${$((OPTIND - 1))-}"
shift $((OPTIND - 1))
if [ $# -gt 0 ] && [ "$SEP" != "--" ]; then
  echo "error: unexpected argument: $1 (bridge options go after --)" >&2
  usage >&2
  exit 1
fi
EXTRA=("$@")

case "$TMO" in
  ''|*[!0-9]*) echo "error: -t needs a whole number of milliseconds, got '$TMO'" >&2; exit 1 ;;
esac

if [ -z "${WOKWI_CLI_TOKEN:-}" ]; then
  echo "error: WOKWI_CLI_TOKEN is not set" >&2
  exit 1
fi

WOKWI="${WOKWI_CLI:-wokwi-cli}"
if ! command -v "$WOKWI" >/dev/null 2>&1; then
  echo "error: $WOKWI not found (put wokwi-cli on PATH or set WOKWI_CLI)" >&2
  exit 1
fi

if [ -z "$FW" ]; then
  echo "error: FW_ROOT is not set (use -f FW_ROOT or set the FW_ROOT environment variable)" >&2
  exit 1
fi
FW=$(abspath "$FW")

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
PROJ="$REPO/programs/ttl_servo"

for w in wokwi-fe-urt-1.chip.wasm wokwi-ttl-servo.chip.wasm; do
  if [ ! -r "$REPO/wokwi/chips/$w" ]; then
    echo "error: chip WASM missing: wokwi/chips/$w (run: make wokwi-chips)" >&2
    exit 1
  fi
done

BRIDGE="$FW/poc/esp32-dof/monitor/dof_wheel_bridge.py"
if [ ! -r "$BRIDGE" ]; then
  echo "error: bridge not found: $BRIDGE (check -f / FW_ROOT)" >&2
  exit 1
fi

FW_BIN="$REPO/.pio/build/ttl_servo-xiao_c3/firmware.bin"
if [ ! -r "$FW_BIN" ]; then
  echo "error: firmware missing: $FW_BIN (build: PLATFORMIO_BUILD_FLAGS=\"-D TTL_SERVO_DOF_FRAMES\" pio run -e ttl_servo-xiao_c3; rebuild without the flag afterwards)" >&2
  exit 1
fi

case "$SCEN" in
  /*) SCEN_ABS=$SCEN ;;
  *)
    if [ -r "$PROJ/$SCEN" ]; then
      SCEN_ABS="$PROJ/$SCEN"
    else
      SCEN_ABS=$(abspath "$SCEN")
    fi
    ;;
esac
if [ ! -r "$SCEN_ABS" ]; then
  echo "error: scenario not found: $SCEN (looked in $PROJ and the current directory)" >&2
  exit 1
fi

case "$WOKWI" in
  */*) WOKWI=$(abspath "$WOKWI") ;;
esac

# Job control gives the background pipeline its own process group, so the signal handler can kill
# wokwi-cli and the bridge together.
set -m
PIPE_PID=""

on_signal() {
  trap - INT TERM
  if [ -n "$PIPE_PID" ]; then
    kill -TERM -- "-$PIPE_PID" 2>/dev/null
    wait "$PIPE_PID" 2>/dev/null
  fi
  echo "stopped on $1; pipeline killed" >&2
  exit "$2"
}
trap 'on_signal INT 130' INT
trap 'on_signal TERM 143' TERM

echo "live: wokwi-cli --timeout $TMO --scenario $SCEN_ABS programs/ttl_servo | python3 $BRIDGE --stdin --core lqr ${EXTRA[*]+"${EXTRA[*]}"}" >&2

(
  cd "$REPO" || exit 1
  "$WOKWI" --timeout "$TMO" --scenario "$SCEN_ABS" programs/ttl_servo < /dev/null \
    | python3 "$BRIDGE" --stdin --core lqr ${EXTRA[@]+"${EXTRA[@]}"}
  W_RC=${PIPESTATUS[0]}
  B_RC=${PIPESTATUS[1]}
  if [ "$B_RC" -ne 0 ]; then
    echo "bridge exit $B_RC" >&2
    exit "$B_RC"
  fi
  if [ "$W_RC" -ne 0 ] && [ "$W_RC" -ne 42 ]; then
    echo "wokwi-cli exit $W_RC" >&2
    exit "$W_RC"
  fi
  if [ "$W_RC" -eq 42 ]; then
    echo "wokwi-cli exit 42 (timeout, expected for the firmware loop)" >&2
  fi
  exit 0
) &
PIPE_PID=$!
wait "$PIPE_PID"
RC=$?
PIPE_PID=""
exit "$RC"
