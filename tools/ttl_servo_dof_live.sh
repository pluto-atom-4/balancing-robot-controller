#!/usr/bin/env bash
# tools/ttl_servo_dof_live.sh
#
# SIM-ONLY, not hardware-validated. Runs the headless Wokwi ttl_servo DOF frame stream (build with
# -D TTL_SERVO_DOF_FRAMES) and pipes wokwi-cli's stdout live into the freecad-workspace bridge
# --stdin. One-way stream by default: the firmware PD drives the Wokwi servo chip; the Python LQR command is
# only shown on the Webots display wheel; nothing feeds back into the physics. With -c the bridge
# command is sent back to the firmware through wokwi-cli --interactive (closed loop, SIM-ONLY).
# Never builds firmware. See -h for usage and exit codes.

set -u

usage() {
  cat <<'EOF'
Usage: tools/ttl_servo_dof_live.sh [-s SCENARIO.yaml] [-t TIMEOUT_MS] [-f FW_ROOT] [-c] [-h] [-- BRIDGE_ARGS...]

SIM-ONLY, not hardware-validated. Live stream: wokwi-cli runs headless on programs/ttl_servo
and its stdout is piped into the freecad-workspace bridge (poc/esp32-dof/monitor/dof_wheel_bridge.py
--stdin --core lqr).
  - Two controllers: the firmware PD (drives the Wokwi servo chip, the physics) and the Python LQR
    (its command is only shown on the Webots display wheel). Without -c: no feedback into the physics
    or the chip. With -c: closed loop, see below.
  - Sign conventions UNVERIFIED (BRC#185, FW#359, FW#377). Not hardware-validated.
  - The Webots display with this stream has not been run (headless runs only, 2026-10-29).

  -s SCENARIO.yaml  scenario; relative paths are looked up in programs/ttl_servo first, then in
                    the current directory (default: scenario_dof_frames.yaml)
  -t TIMEOUT_MS     wokwi-cli timeout in ms (default: 60000)
  -f FW_ROOT        freecad-workspace repo root (default: the FW_ROOT environment variable;
                    one of the two is required)
  -c                closed loop: the bridge command is sent back to the firmware through wokwi-cli
                    --interactive (needs the firmware built with -D TTL_SERVO_DOF_FRAMES -D TTL_SERVO_EXT_CMD,
                    pluto-atom-4/balancing-robot-controller#221; the script cannot check the flag). Use
                    -- --cmd-scale -1 if the loop diverges: the sign of the Python LQR command versus the
                    firmware sim is UNVERIFIED (pluto-atom-4/freecad-workspace#359,
                    pluto-atom-4/balancing-robot-controller#185). The stream stops when the sim body falls.
                    Without the firmware flag the commands are ignored and the run is open loop.
                    Run length with -c: wokwi-cli --interactive does NOT exit after a scenario has completed (it
                    waits for stdin to close, which never happens here), so use a scenario that never completes
                    (scenario_dof_closed.yaml) with an explicit -t; the run then ends at -t with exit 42, which
                    counts as normal. A completed scenario would leave the run waiting (observed 2026-10-29).
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

Run length: about 15 s of wall time for the default scenario (scenario_dof_frames.yaml): about 8 s until
the first frame, then the scenario steps (about 7 s). wokwi-cli then exits 0 and the bridge sees end of
input. -t only matters if the scenario does not finish (for example a build without the flag never prints
the banner the scenario waits for: wokwi-cli exits 42 after -t). In that case the bridge gets no frames
and exits 2 ("never armed"), which this script returns. Exit 0 does NOT prove the scenario finished: a
-t shorter than the run gives wokwi-cli exit 42, one stderr line, and exit 0 if the bridge exited 0.
Ctrl-C is meant to stop wokwi-cli and the bridge together (same foreground process group); this was NOT
run. A SIGTERM or SIGHUP sent only to this script is not forwarded to them.
The bridge sends UDP to 127.0.0.1:5006 (its default), the port the Webots
servo_wheel_dof world listens on. Pass `-- --udp-port N` to change it.

Exit codes. The bridge's and wokwi-cli's own exit codes can collide with the codes below (in particular a
bridge exit 1 cannot be told apart from this script's own exit 1).
  0    the bridge exited 0 and wokwi-cli exited 0 or 42
  1    usage, input or environment error (one error line on stderr; option and argument errors also print this text)
  N    the bridge's nonzero exit code N (checked first; 2 = never armed, for example no frames)
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

cleanup() {
  exec 3>&- 2>/dev/null
  rm -f "$CMD_FIFO" 2>/dev/null
  rmdir "$CMD_DIR" 2>/dev/null
  return 0
}

SCEN="scenario_dof_frames.yaml"
TMO="60000"
FW="${FW_ROOT:-}"
CLOSED=0
CMD_DIR=""
CMD_FIFO=""

while getopts ":s:t:f:ch" opt; do
  case "$opt" in
    s) SCEN=$OPTARG ;;
    t) TMO=$OPTARG ;;
    f) FW=$OPTARG ;;
    c) CLOSED=1 ;;
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

if ! command -v python3 >/dev/null 2>&1; then
  echo "error: python3 not found on PATH (the FW bridge needs it; try: mamba run -n esp32-dof $0 ...)" >&2
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

WOKWI_OPTS=(--timeout "$TMO" --scenario "$SCEN_ABS")
BRIDGE_OPTS=(--stdin --core lqr)
IN_FILE=/dev/null
if [ "$CLOSED" -eq 1 ]; then
  CMD_DIR=$(mktemp -d) || { echo "error: mktemp -d failed" >&2; exit 1; }
  trap 'cleanup' EXIT
  trap 'cleanup; exit 130' INT
  trap 'cleanup; exit 143' TERM
  trap 'cleanup; exit 129' HUP
  CMD_FIFO="$CMD_DIR/cmd.fifo"
  mkfifo "$CMD_FIFO" || { echo "error: mkfifo failed: $CMD_FIFO" >&2; exit 1; }
  # Hold the FIFO open read-write in this script (fd 3) so that neither side's open can block forever:
  # if the bridge dies before it opens the FIFO, wokwi-cli's input redirect still succeeds. A side effect
  # is that wokwi-cli never sees end of input on stdin, which it does not need.
  exec 3<>"$CMD_FIFO" || { echo "error: cannot open the FIFO: $CMD_FIFO" >&2; exit 1; }
  WOKWI_OPTS+=(--interactive)
  BRIDGE_OPTS+=(--cmd-out "$CMD_FIFO")
  IN_FILE=$CMD_FIFO
fi

# Informational only: omits the input redirect (< /dev/null, or < FIFO with -c) and does not quote the arguments.
if [ "$CLOSED" -eq 1 ]; then
  echo "live: wokwi-cli --timeout $TMO --scenario $SCEN_ABS --interactive programs/ttl_servo < $CMD_FIFO | python3 $BRIDGE --stdin --core lqr --cmd-out $CMD_FIFO ${EXTRA[*]+"${EXTRA[*]}"}" >&2
else
  echo "live: wokwi-cli --timeout $TMO --scenario $SCEN_ABS programs/ttl_servo | python3 $BRIDGE --stdin --core lqr ${EXTRA[*]+"${EXTRA[*]}"}" >&2
fi

# Foreground pipeline on purpose: Ctrl-C reaches wokwi-cli and the bridge together (one process group),
# so no signal trap is needed. A SIGTERM or SIGHUP sent only to this script is NOT forwarded.
cd "$REPO" || exit 1
"$WOKWI" "${WOKWI_OPTS[@]}" programs/ttl_servo < "$IN_FILE" \
  | python3 "$BRIDGE" "${BRIDGE_OPTS[@]}" ${EXTRA[@]+"${EXTRA[@]}"}
PS=("${PIPESTATUS[@]}")   # copy at once: any later command resets PIPESTATUS
W_RC=${PS[0]}
B_RC=${PS[1]}
if [ "$B_RC" -ne 0 ]; then
  echo "bridge exit $B_RC" >&2
  exit "$B_RC"
fi
if [ "$W_RC" -ne 0 ] && [ "$W_RC" -ne 42 ]; then
  echo "wokwi-cli exit $W_RC" >&2
  exit "$W_RC"
fi
if [ "$W_RC" -eq 42 ]; then
  echo "wokwi-cli exit 42 (timeout; the scenario did not finish before -t)" >&2
fi
exit 0
