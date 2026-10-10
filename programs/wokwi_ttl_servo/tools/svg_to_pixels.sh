#!/usr/bin/env bash
# Convert an SVG to a 64x64 C array of uint32_t pixels in the format the
# wokwi-ttl-servo chip writes to its framebuffer: 0xAABBGGRR (R in the low
# byte). This pixel format was confirmed in the Wokwi simulator by the repo
# owner (colours correct); if a different front end shows swapped colours,
# rerun with --bgr.
#
# Needs: inkscape (rasterizes SVG incl. <style> classes) and ImageMagick
# (magick or convert), plus od and awk.
#
# Usage:
#   svg_to_pixels.sh [--size N] [--hide-id ID]... [--flatten COLOR|none] [--bgr] [--name SYMBOL] in.svg [out.h]
#   --size N         output is N x N pixels (default 64; chip display is 64x64)
#   --hide-id ID     hide the SVG element with this id before rendering, e.g.
#                    `horn` to bake only the static servo body (the chip draws
#                    the rotating horn itself). Repeatable: each ID is hidden.
#                    Exits 1 if Inkscape (--query-all) reports no element with that id.
#   --flatten COLOR  after downscale, composite onto COLOR (default #202020) so
#                    every pixel is opaque (alpha 0xFF). `none` keeps transparency.
#   --bgr            swap R and B (use if red shows up blue in the GUI)
#   --name SYMBOL    C array name (default: servo_bg_pixels)
# Without out.h the array is printed to stdout.
# Exit codes: 0 success; 2 usage/option error (bad option, missing value,
# invalid --size or --name); 1 runtime failure (missing file or tool, unknown
# --hide-id, invalid --flatten colour, bad out path, render or write failure).
set -euo pipefail

size=64
hide_ids=()
flatten="#202020"
swap=0
name="servo_bg_pixels"

while [ $# -gt 0 ]; do
  case "$1" in
    --size) [ $# -ge 2 ] || { echo "$1 needs a value" >&2; exit 2; }; size="$2"; shift 2 ;;
    --hide-id) [ $# -ge 2 ] || { echo "$1 needs a value" >&2; exit 2; }; hide_ids+=("$2"); shift 2 ;;
    --flatten) [ $# -ge 2 ] || { echo "$1 needs a value" >&2; exit 2; }; flatten="$2"; shift 2 ;;
    --bgr) swap=1; shift ;;
    --name) [ $# -ge 2 ] || { echo "$1 needs a value" >&2; exit 2; }; name="$2"; shift 2 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d'; exit 0 ;;
    --) shift; break ;;
    -*) echo "unknown option: $1" >&2; exit 2 ;;
    *) break ;;
  esac
done

in="${1:-}"
out="${2:-}"
[ -n "$in" ] || { echo "usage: $0 [options] in.svg [out.h]" >&2; exit 2; }
[[ "$size" =~ ^[1-9][0-9]*$ ]] || { echo "--size must be a positive integer: $size" >&2; exit 2; }
[[ "$name" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || { echo "--name must be a C identifier: $name" >&2; exit 2; }
[ -f "$in" ] || { echo "no such file: $in" >&2; exit 1; }
if [ -n "$out" ]; then
  [ ! -d "$out" ] || { echo "out is a directory: $out" >&2; exit 1; }
  [ -d "$(dirname "$out")" ] || { echo "output directory does not exist: $(dirname "$out")" >&2; exit 1; }
fi
command -v inkscape >/dev/null || { echo "inkscape not found" >&2; exit 1; }
if command -v magick >/dev/null; then im=(magick); else im=(convert); fi
command -v "${im[0]}" >/dev/null || { echo "ImageMagick not found" >&2; exit 1; }

tmp="$(mktemp -d)"
tmp_out=""
trap 'rm -rf "$tmp"; [ -z "${tmp_out:-}" ] || rm -f "$tmp_out"' EXIT

# 0. Validate --hide-id against Inkscape's own element list, before rasterizing.
if [ "${#hide_ids[@]}" -gt 0 ]; then
  inkscape --query-all "$in" 2>/dev/null | cut -d, -f1 > "$tmp/ids.txt" \
    || { echo "inkscape --query-all failed on $in" >&2; exit 1; }
  for id in "${hide_ids[@]}"; do
    grep -qxF -- "$id" "$tmp/ids.txt" || { echo "no element with id=\"$id\" in $in" >&2; exit 1; }
  done
fi

# 0b. Validate --flatten colour (skip for none): ImageMagick must accept it and
#     produce exactly one RGBA pixel (4 bytes).
if [ "$flatten" != "none" ]; then
  if ! { "${im[@]}" -size 1x1 "xc:$flatten" -depth 8 "rgba:$tmp/col.raw" >/dev/null 2>&1 \
         && [ "$(wc -c < "$tmp/col.raw")" -eq 4 ]; }; then
    echo "invalid --flatten colour: $flatten" >&2
    exit 1
  fi
fi

# 1. Rasterize at 4x, then downscale with a box filter (cleaner edges than
#    rendering straight to 64 px). Transparent background stays transparent
#    unless --flatten is used (step 2).
big=$((size * 4))
actions=""
if [ "${#hide_ids[@]}" -gt 0 ]; then
  for id in "${hide_ids[@]}"; do
    actions="${actions}select-by-id:$id;object-set-attribute:style,display:none;select-clear;"
  done
fi
actions="${actions}export-type:png;export-filename:$tmp/big.png;export-width:$big;export-height:$big;export-background-opacity:0;export-do"
rc=0
inkscape --actions="$actions" "$in" >/dev/null 2>"$tmp/inkscape.log" || rc=$?
if [ ! -f "$tmp/big.png" ]; then
  echo "inkscape produced no PNG (exit $rc); last log lines:" >&2
  tail -n 20 "$tmp/inkscape.log" >&2
  exit 1
fi

# 2. Downscale to size x size; optionally flatten onto a solid colour so the
#    alpha channel is forced to 0xFF. Dump raw 8-bit RGBA bytes.
flat_on=0
if [ "$flatten" = "none" ]; then
  "${im[@]}" "$tmp/big.png" -filter Box -resize "${size}x${size}!" -depth 8 "rgba:$tmp/px.raw"
else
  flat_on=1
  "${im[@]}" "$tmp/big.png" -filter Box -resize "${size}x${size}!" \
    -background "$flatten" -alpha remove -alpha off \
    -alpha on -channel A -evaluate set 100% +channel \
    -depth 8 "rgba:$tmp/px.raw"
fi
bytes=$(wc -c < "$tmp/px.raw")
[ "$bytes" -eq $((size * size * 4)) ] || { echo "unexpected raw size: $bytes" >&2; exit 1; }

# 3. Format as 0xAABBGGRR (little-endian uint32 of bytes R,G,B,A).
#    With flattening, every alpha byte must be 255 or the script fails.
emit() {
  echo "// GENERATED FILE - DO NOT EDIT. Regenerate with: make wokwi_ttl_servo-bg"
  echo "// Generated by programs/wokwi_ttl_servo/tools/svg_to_pixels.sh from $(basename "$in")"
  echo "// ${size}x${size}, 0xAABBGGRR$([ "$swap" -eq 1 ] && echo ' (R/B swapped)'). Pixel format confirmed in the Wokwi simulator."
  echo "#include <stdint.h>"
  echo
  echo "static const uint32_t ${name}[${size} * ${size}] = {"
  od -An -v -tu1 -w4 "$tmp/px.raw" | awk -v swap="$swap" -v chk="$flat_on" '
    { r=$1; g=$2; b=$3; a=$4
      if (chk && a != 255) bad=1
      if (swap) { t=r; r=b; b=t }
      printf "0x%02X%02X%02X%02X,", a, b, g, r
      if (NR % 8 == 0) printf "\n"; else printf " " }
    END {
      if (bad) { print "error: non-opaque pixel after --flatten" > "/dev/stderr"; exit 1 }
      if (NR % 8 != 0) printf "\n" }' || return 1
  echo "};"
}

if [ -n "$out" ]; then
  # Temp file beside the target so mv is an atomic rename on the same filesystem.
  tmp_out="$(mktemp "$(dirname "$out")/.$(basename "$out").XXXXXX")"
  if ! emit > "$tmp_out"; then
    rm -f "$tmp_out"
    echo "failed to generate header; $out left unchanged" >&2
    exit 1
  fi
  chmod 0644 "$tmp_out"
  mv -f "$tmp_out" "$out"
  echo "wrote $out" >&2
else
  emit
fi
