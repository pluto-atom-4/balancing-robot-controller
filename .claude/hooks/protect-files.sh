#!/usr/bin/env bash
# PreToolUse guard for Edit|Write: blocks edits to generated, human-approval and guard files.
# Protected path suffixes: lib/balance_core/balance_gains_generated.h,
# test/test_balance_parity/vectors_generated.inc, wokwi/chips/wokwi-ttl-servo.bg.h,
# .claude/settings.json, .claude/settings.local.json, .claude/hooks/protect-files.sh, CLAUDE.md.
# Reads the tool input JSON on stdin, exits 2 with a stderr message on a match, else 0.
# Both the raw path and the `realpath -m` form are checked, so `..`, `//` and symlinked
# parent directories do not slip past the suffix match.
# FAIL-OPEN: missing jq, bad JSON or any error exits 0 (jq is a hard requirement for the guard).
# KNOWN GAPS: only the Edit and Write tools are matched (settings.json matcher `Edit|Write`).
# Bash writes (sed -i, tee, cp), NotebookEdit and MultiEdit are NOT covered, so this is a
# speed bump, not a security boundary. It applies to the main session too and there is no
# in-repo bypass: the owner applies edits to protected files outside Claude Code, or
# removes the hook entry. Limitation: any path ending in /CLAUDE.md matches.
command -v jq >/dev/null 2>&1 || exit 0
input=$(cat) || exit 0
fp=$(printf '%s' "$input" | jq -r '.tool_input.file_path // empty' 2>/dev/null) || exit 0
[ -n "$fp" ] || exit 0
rp=$(realpath -m -- "$fp" 2>/dev/null) || rp=$fp
for cand in "$fp" "$rp"; do
  for p in lib/balance_core/balance_gains_generated.h \
           test/test_balance_parity/vectors_generated.inc \
           wokwi/chips/wokwi-ttl-servo.bg.h \
           .claude/settings.json \
           .claude/settings.local.json \
           .claude/hooks/protect-files.sh \
           CLAUDE.md; do
    case "$cand" in
      "$p"|*/"$p")
        case "$p" in
          wokwi/chips/wokwi-ttl-servo.bg.h) hint="regenerate it with 'make wokwi-bg'." ;;
          lib/balance_core/balance_gains_generated.h|test/test_balance_parity/vectors_generated.inc)
            hint="regenerate it with the Python exporter (see CLAUDE.md)." ;;
          *) hint="ask the human; the owner applies this edit." ;;
        esac
        echo "protect-files: $fp is a protected file: $hint" >&2
        exit 2
        ;;
    esac
  done
done
exit 0
