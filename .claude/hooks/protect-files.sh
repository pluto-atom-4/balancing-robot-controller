#!/usr/bin/env bash
# PreToolUse guard for Edit|Write: blocks edits to generated and human-approval files.
# Protected path suffixes: lib/balance_core/balance_gains_generated.h,
# test/test_balance_parity/vectors_generated.inc, wokwi/chips/wokwi-ttl-servo.bg.h,
# .claude/settings.json, .claude/settings.local.json, CLAUDE.md.
# Reads the tool input JSON on stdin, exits 2 with a stderr message on a match, else 0.
# FAIL-OPEN: missing jq, bad JSON or any error exits 0. Applies to the main session too,
# so human-approved edits to protected files need the owner to apply them or to
# bypass this hook temporarily. Limitation: any path ending in /CLAUDE.md matches.
command -v jq >/dev/null 2>&1 || exit 0
input=$(cat) || exit 0
fp=$(printf '%s' "$input" | jq -r '.tool_input.file_path // empty' 2>/dev/null) || exit 0
[ -n "$fp" ] || exit 0
for p in lib/balance_core/balance_gains_generated.h \
         test/test_balance_parity/vectors_generated.inc \
         wokwi/chips/wokwi-ttl-servo.bg.h \
         .claude/settings.json \
         .claude/settings.local.json \
         CLAUDE.md; do
  case "$fp" in
    "$p"|*/"$p")
      echo "protect-files: $fp is a protected file. Generated files: regenerate with the Python exporter. settings.json / CLAUDE.md: ask the human." >&2
      exit 2
      ;;
  esac
done
exit 0
