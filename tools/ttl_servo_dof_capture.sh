#!/usr/bin/env bash
# tools/ttl_servo_dof_capture.sh
#
# SIM-ONLY helper for the ttl_servo DOF frame stream (epic #184, S4 #189).
# Not hardware-validated. Nothing here runs on a robot.
#
# WHAT IT DOES
#   1. Capture mode (default): runs wokwi-cli headless on programs/ttl_servo with a scenario
#      (default scenario_dof_frames.yaml), writes the serial log to LOG, then converts it.
#   2. Convert-only mode (-c EXISTING.log): skips wokwi-cli and converts an existing serial log.
#   3. Conversion: keeps only complete 'IMU,' frame lines (CR stripped, shape checked by a
#      regex), strips the 'IMU,' prefix and the '*HH' suffix, and writes OUT.csv with the header
#      seq,t_us,ax,ay,az,gx,gy,gz. That is the RAW CSV the FW dof_wheel_bridge --replay reads.
#   4. Prints rows kept, rows dropped, first and last seq to stderr. If fewer than 50 rows are
#      kept it warns and exits 3 (the bridge needs 50 frames to arm; 49 exits 2).
#
# WHAT IT DOES NOT DO
#   - It does NOT build firmware. The caller builds first (see the sequence below). Capture mode
#     uses whatever is in .pio/build/ttl_servo-xiao_c3, so a build without the flag gives no
#     'IMU,' lines and the scenario times out.
#   - It does NOT verify checksums. The *HH suffix is stripped, not checked. To check a capture,
#     parse it with the FW dof_frame.parse_line (freecad-workspace, poc/esp32-dof).
#   - It does NOT run Webots or the FW bridge.
#   - A frame split by wokwi-cli's own log lines fails the regex and is dropped. The summary
#     counts it as dropped (prefix 'IMU,' lines minus kept rows).
#
# COMMAND SEQUENCE
#   a. export WOKWI_CLI_TOKEN=...          (capture mode only)
#   b. PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" pio run -e ttl_servo-xiao_c3
#   c. tools/ttl_servo_dof_capture.sh -o /abs/path/ttl_servo_dof_raw.csv
#   d. In the FW repo (freecad-workspace), NOT run by this script and not verified here:
#        SWD_UDP_PORT=5006 ./run_demo.sh      (in poc/servo-wheel-dof; starts Webots)
#        mamba run -n esp32-dof python3 poc/esp32-dof/monitor/dof_wheel_bridge.py \
#          --replay /abs/path/ttl_servo_dof_raw.csv --core lqr --udp-port 5006
#   e. Shared build dir: .pio/build/ttl_servo-xiao_c3 now holds the DOF build. Rebuild it
#      without the flag (pio run -e ttl_servo-xiao_c3) before any normal sim, and rebuild the
#      default env (pio run) when you are done.
#
# Usage and exit codes: run with -h.

set -u

usage() {
  cat <<'EOF'
Usage: tools/ttl_servo_dof_capture.sh [-o OUT.csv] [-s SCENARIO.yaml] [-t TIMEOUT_MS] [-l LOG] [-c EXISTING.log]

  -o OUT.csv        raw bridge CSV to write (default: ttl_servo_dof_raw.csv)
  -s SCENARIO.yaml  scenario file; relative paths are looked up in programs/ttl_servo first,
                    then in the current directory (default: scenario_dof_frames.yaml)
  -t TIMEOUT_MS     wokwi-cli timeout in milliseconds (default: 20000)
  -l LOG            serial log written by wokwi-cli (default: OUT.csv with .log appended)
  -c EXISTING.log   convert-only mode: skip wokwi-cli, convert this serial log.
                    -s, -t and -l are ignored in this mode.
  -h                show this help

Capture mode needs WOKWI_CLI_TOKEN set, and the firmware already built with
PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" pio run -e ttl_servo-xiao_c3.
The wokwi-cli binary is taken from $WOKWI_CLI, else wokwi-cli on PATH.

Exit codes:
  0  capture passed (wokwi-cli exit 0) and at least 50 rows
  3  fewer than 50 rows (bridge would not arm)
  1  usage, input or environment error
  N  wokwi-cli's own nonzero exit code (for example 42 on timeout); the log is still converted
EOF
}

# Frame shape: seq, t_us, six decimal values (ax ay az gx gy gz), then '*HH' (two hex digits).
# Checksums are not verified here.
FRAME_RE='^IMU,[0-9]+,[0-9]+,(-?[0-9]+\.[0-9]+,){5}-?[0-9]+\.[0-9]+\*[0-9A-F]{2}$'
HEADER='seq,t_us,ax,ay,az,gx,gy,gz'
MIN_ROWS=50

OUT="ttl_servo_dof_raw.csv"
SCEN="scenario_dof_frames.yaml"
TMO="20000"
LOG=""
SRC_LOG=""

while getopts ":o:s:t:l:c:h" opt; do
  case "$opt" in
    o) OUT=$OPTARG ;;
    s) SCEN=$OPTARG ;;
    t) TMO=$OPTARG ;;
    l) LOG=$OPTARG ;;
    c) SRC_LOG=$OPTARG ;;
    h) usage; exit 0 ;;
    :) echo "error: -$OPTARG needs a value" >&2; usage >&2; exit 1 ;;
    *) echo "error: unknown option -$OPTARG" >&2; usage >&2; exit 1 ;;
  esac
done
shift $((OPTIND - 1))
if [ $# -gt 0 ]; then
  echo "error: unexpected argument: $1" >&2
  usage >&2
  exit 1
fi

# Turn a path into an absolute one, relative to the caller's directory.
abspath() {
  case "$1" in
    /*) printf '%s\n' "$1" ;;
    *) printf '%s/%s\n' "$PWD" "$1" ;;
  esac
}

OUT=$(abspath "$OUT")
if [ -z "$LOG" ]; then
  LOG="$OUT.log"
fi
LOG=$(abspath "$LOG")

case "$TMO" in
  ''|*[!0-9]*) echo "error: -t needs a whole number of milliseconds, got '$TMO'" >&2; exit 1 ;;
esac

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
PROJ="$REPO/programs/ttl_servo"

# convert SRC DST: sets KEPT, prints the summary to stderr. Returns 1 on a write error.
KEPT=0
convert() {
  local src=$1 dst=$2 prefix dropped first last
  prefix=$(tr -d '\r' < "$src" | grep -c '^IMU,')
  {
    printf '%s\n' "$HEADER"
    tr -d '\r' < "$src" | grep -E "$FRAME_RE" \
      | sed -e 's/^IMU,//' -e 's/\*[0-9A-F][0-9A-F]$//'
  } > "$dst" || return 1

  KEPT=$(( $(wc -l < "$dst") - 1 ))
  if [ "$KEPT" -gt 0 ]; then
    first=$(sed -n '2p' "$dst" | cut -d, -f1)
    last=$(tail -n 1 "$dst" | cut -d, -f1)
  else
    first="-"
    last="-"
  fi
  dropped=$(( prefix - KEPT ))

  echo "convert: $src -> $dst" >&2
  echo "  rows kept:    $KEPT (header excluded)" >&2
  echo "  rows dropped: $dropped ('IMU,' lines that failed the frame shape)" >&2
  echo "  first seq:    $first" >&2
  echo "  last seq:     $last" >&2
  return 0
}

check_rows() {
  if [ "$1" -lt "$MIN_ROWS" ]; then
    echo "WARNING: only $1 rows. The FW bridge --replay needs at least $MIN_ROWS frames to arm (49 exits 2)." >&2
    return 3
  fi
  return 0
}

# Convert-only mode: no wokwi-cli, no token.
if [ -n "$SRC_LOG" ]; then
  SRC_LOG=$(abspath "$SRC_LOG")
  if [ ! -r "$SRC_LOG" ]; then
    echo "error: cannot read serial log: $SRC_LOG" >&2
    exit 1
  fi
  echo "convert-only mode: wokwi-cli is not run" >&2
  if ! convert "$SRC_LOG" "$OUT"; then
    echo "error: cannot write $OUT" >&2
    exit 1
  fi
  check_rows "$KEPT"
  exit $?
fi

# Capture mode.
if [ -z "${WOKWI_CLI_TOKEN:-}" ]; then
  echo "error: WOKWI_CLI_TOKEN is not set (capture mode needs it; use -c to convert an existing log)" >&2
  exit 1
fi

WOKWI="${WOKWI_CLI:-wokwi-cli}"
if ! command -v "$WOKWI" >/dev/null 2>&1; then
  echo "error: $WOKWI not found (put wokwi-cli on PATH or set WOKWI_CLI)" >&2
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

echo "capture: wokwi-cli --timeout $TMO --scenario $SCEN_ABS --serial-log-file $LOG $PROJ" >&2
( cd "$REPO" && "$WOKWI" --timeout "$TMO" --scenario "$SCEN_ABS" --serial-log-file "$LOG" "$PROJ" )
RC=$?

if [ "$RC" -eq 0 ]; then
  echo "wokwi-cli: scenario passed (exit 0)" >&2
else
  echo "wokwi-cli: exit $RC (42 = timeout or no match; check the flag and the scenario). Converting the log anyway." >&2
fi

if [ ! -r "$LOG" ]; then
  echo "error: serial log not written: $LOG" >&2
  if [ "$RC" -ne 0 ]; then exit "$RC"; fi
  exit 1
fi

if ! convert "$LOG" "$OUT"; then
  echo "error: cannot write $OUT" >&2
  exit 1
fi
check_rows "$KEPT"
ROWS_RC=$?

if [ "$RC" -ne 0 ]; then
  exit "$RC"
fi
exit "$ROWS_RC"
