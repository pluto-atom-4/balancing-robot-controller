#!/usr/bin/env bash
# tools/ttl_servo_dof_capture.sh
#
# SIM-ONLY, not hardware-validated. Converts the serial log of a headless Wokwi run of
# programs/ttl_servo (DOF frame stream) to the raw CSV the FW bridge --replay reads.
# No firmware build, no checksum check, no Webots. See -h for usage and exit codes.

set -u

usage() {
  cat <<'EOF'
Usage: tools/ttl_servo_dof_capture.sh [-o OUT.csv] [-s SCENARIO.yaml] [-t TIMEOUT_MS] [-l LOG] [-c EXISTING.log]

Runs wokwi-cli headless on programs/ttl_servo and converts its serial log to a raw bridge CSV
(header seq,t_us,ax,ay,az,gx,gy,gz). With -c, converts an existing serial log instead.

  -o OUT.csv        CSV to write (default: ttl_servo_dof_raw.csv)
  -s SCENARIO.yaml  scenario; relative paths are looked up in programs/ttl_servo first, then in
                    the current directory (default: scenario_dof_frames.yaml)
  -t TIMEOUT_MS     capture mode only: wokwi-cli timeout in ms (default: 20000)
  -l LOG            capture mode only: serial log path (default: OUT.csv with .log appended).
                    An existing LOG is moved to LOG.prev before the run.
  -c EXISTING.log   convert-only mode: skip wokwi-cli and the chip check; -s, -t, -l are ignored
  -h                show this help

Capture mode, in order:
  1. Build first: PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" pio run -e ttl_servo-xiao_c3
  2. make wokwi-chips   (only if the chip .wasm files are missing; needs network on first run)
  3. WOKWI_CLI_TOKEN=... tools/ttl_servo_dof_capture.sh -o OUT.csv
  Afterwards rebuild without the flag: pio run -e ttl_servo-xiao_c3 (shared .pio build dir).

The script never builds firmware. WOKWI_CLI selects the wokwi-cli binary (default: wokwi-cli on PATH).

Exit codes. wokwi-cli's own exit codes (for example 1 or 3) can collide with the codes below.
  0  capture passed (wokwi-cli exit 0) and at least 50 rows; convert-only: at least 50 rows
  3  fewer than 50 rows
  1  usage, input or environment error, missing chip .wasm, or the CSV could not be written
  N  wokwi-cli's own nonzero exit code (for example 42 on timeout); the log is still converted.
     If the CSV then cannot be written, the script exits 1 and says that wokwi-cli exited N.
EOF
}

# Frame shape: seq, t_us, six decimal values (ax ay az gx gy gz), then '*HH' (two uppercase hex
# digits). Checksums are not verified here.
FRAME_RE='^IMU,[0-9]+,[0-9]+,(-?[0-9]+\.[0-9]+,){5}-?[0-9]+\.[0-9]+\*[0-9A-F]{2}$'
HEADER='seq,t_us,ax,ay,az,gx,gy,gz'
MIN_ROWS=50
CHIP_WASMS='wokwi-fe-urt-1.chip.wasm wokwi-ttl-servo.chip.wasm'

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

# same_file A B: true when both names are the same path (or hard links to one existing file).
same_file() {
  [ "$1" = "$2" ] && return 0
  [ -e "$2" ] && [ "$1" -ef "$2" ]
}

OUT=$(abspath "$OUT")
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
PROJ="$REPO/programs/ttl_servo"

KEPT=0
CONV_TMP=""

# convert SRC: writes the CSV to a temp file next to OUT, prints the summary, then moves the temp
# to OUT. The temp is left in place (and its path printed) if a write or the move fails.
convert() {
  local src=$1 prefix dropped first last
  prefix=$(tr -d '\r' < "$src" | grep -c '^IMU,')

  CONV_TMP=$(mktemp "$OUT.XXXXXX") || { echo "error: cannot create a temp file next to $OUT" >&2; return 1; }
  {
    printf '%s\n' "$HEADER"
    tr -d '\r' < "$src" | grep -E "$FRAME_RE" \
      | sed -e 's/^IMU,//' -e 's/\*[0-9A-F][0-9A-F]$//'
  } > "$CONV_TMP" || { echo "error: write failed; temp file left at $CONV_TMP" >&2; return 1; }

  KEPT=$(( $(wc -l < "$CONV_TMP") - 1 ))
  if [ "$KEPT" -gt 0 ]; then
    first=$(sed -n '2p' "$CONV_TMP" | cut -d, -f1)
    last=$(tail -n 1 "$CONV_TMP" | cut -d, -f1)
  else
    first="-"
    last="-"
  fi
  dropped=$(( prefix - KEPT ))

  echo "convert: $src -> $OUT" >&2
  echo "  rows kept:    $KEPT (header excluded)" >&2
  echo "  IMU lines dropped: at least $dropped (IMU-prefixed lines that failed the frame shape;" >&2
  echo "                tail fragments of split frames are not counted)" >&2
  echo "  first seq:    $first" >&2
  echo "  last seq:     $last" >&2

  chmod 644 "$CONV_TMP" 2>/dev/null
  if mv -f "$CONV_TMP" "$OUT"; then
    echo "wrote: $OUT" >&2
    return 0
  fi
  echo "error: could not move the CSV into place; temp file left at $CONV_TMP" >&2
  return 1
}

check_rows() {
  if [ "$1" -lt "$MIN_ROWS" ]; then
    echo "WARNING: only $1 rows. The FW bridge needs $MIN_ROWS frames to arm (per its README; not verified here)." >&2
    return 3
  fi
  return 0
}

# Convert-only mode: no wokwi-cli, no token, no chip check.
if [ -n "$SRC_LOG" ]; then
  SRC_LOG=$(abspath "$SRC_LOG")
  if [ ! -r "$SRC_LOG" ]; then
    echo "error: cannot read serial log: $SRC_LOG" >&2
    exit 1
  fi
  if same_file "$SRC_LOG" "$OUT"; then
    echo "error: -c and -o name the same file ($OUT); refusing to overwrite the input" >&2
    exit 1
  fi
  echo "convert-only mode: wokwi-cli is not run" >&2
  if ! convert "$SRC_LOG"; then
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

case "$TMO" in
  ''|*[!0-9]*) echo "error: -t needs a whole number of milliseconds, got '$TMO'" >&2; exit 1 ;;
esac

WOKWI="${WOKWI_CLI:-wokwi-cli}"
if ! command -v "$WOKWI" >/dev/null 2>&1; then
  echo "error: $WOKWI not found (put wokwi-cli on PATH or set WOKWI_CLI)" >&2
  exit 1
fi

for w in $CHIP_WASMS; do
  if [ ! -r "$REPO/wokwi/chips/$w" ]; then
    echo "error: chip WASM missing: wokwi/chips/$w" >&2
    echo "run: make wokwi-chips" >&2
    exit 1
  fi
done

if [ -z "$LOG" ]; then
  LOG="$OUT.log"
fi
LOG=$(abspath "$LOG")
if same_file "$LOG" "$OUT"; then
  echo "error: -l and -o name the same file ($OUT); refusing to overwrite the CSV with the log" >&2
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

# Never convert a stale log as a fresh capture: move any old one aside first.
if [ -e "$LOG" ]; then
  if ! mv -f "$LOG" "$LOG.prev"; then
    echo "error: could not move the old log aside: $LOG" >&2
    exit 1
  fi
  echo "moved the old log to $LOG.prev" >&2
fi

echo "capture: wokwi-cli --timeout $TMO --scenario $SCEN_ABS --serial-log-file $LOG $PROJ" >&2
( cd "$REPO" && "$WOKWI" --timeout "$TMO" --scenario "$SCEN_ABS" --serial-log-file "$LOG" "$PROJ" )
RC=$?

if [ "$RC" -eq 0 ]; then
  echo "wokwi-cli: scenario passed (exit 0)" >&2
else
  echo "wokwi-cli: exit $RC (42 = timeout: the scenario did not finish). Converting the log anyway." >&2
fi

if [ ! -r "$LOG" ]; then
  echo "error: serial log not written: $LOG" >&2
  if [ "$RC" -ne 0 ]; then exit "$RC"; fi
  exit 1
fi

if ! convert "$LOG"; then
  if [ "$RC" -ne 0 ]; then
    echo "error: the CSV could not be written, and wokwi-cli had exited $RC" >&2
  fi
  exit 1
fi
check_rows "$KEPT"
ROWS_RC=$?

if [ "$RC" -ne 0 ]; then
  exit "$RC"
fi
exit "$ROWS_RC"
