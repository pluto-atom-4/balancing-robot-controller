# Cavecrew rules

Text rules for repo cavecrew agents. Untested until smoke test #126.

## Agents

- `architect`: plans, files issues. Graph + GitHub tools. No code edits.
- `repo-cavecrew-builder`: haiku. 1-2 file edits. Read/Grep/Glob/Edit/Write + graph MCP read tools. No Bash.
- `repo-cavecrew-reviewer`: haiku. Read-only diff review. Input: `base: <ref>` label or pasted diff; anything else is ambiguous.
- `slop-hunter`: haiku. Read-only dead-code and doc-slop reporter. Bash only: `code-review-graph dead-code`, `code-review-graph dead-code --json`, `code-review-graph architecture`, `code-review-graph large-functions`.
- Plugin agents `caveman:cavecrew-builder` and `caveman:cavecrew-reviewer` are separate agents. Project files do not extend them; these rules reach them only through CLAUDE.md and (later) hooks.
- Older local agents `builder`, `reviewer`, `investigator` still exist and are separate.

## Defaults

- Builder edits 1-2 files per run.
- Reviewer and slop-hunter are read-only.
- Model `haiku` for builder, reviewer, slop-hunter.
- Main session runs git, builds, tests. Builders have no shell.

## Handover

- Files changed, with line ranges.
- Commands run, with real output.
- Not-run list.
- Open questions.
- Never claim a test passed unless it was run.

## Gates

- Generated files, not edited: `lib/balance_core/balance_gains_generated.h`, `test/test_balance_parity/vectors_generated.inc`, `wokwi/chips/wokwi-ttl-servo.bg.h`. Edit only if the issue explicitly says so.
- `.claude/settings.json`, `.claude/settings.local.json`, `CLAUDE.md`: not edited unless the issue explicitly says so. `settings.json` edits only in the single human-reviewed PR.
- No weakening of kill paths or the torque-off-first order.
- Safety constants in `programs/balance/config.h` need a design issue.
- `lib/` headers must be C++11-valid (single-return `constexpr`, no `switch` in `constexpr`).
- Build a firmware env after any `lib/` edit.
- `pio test -e test` (native, gnu++17) does not catch C++11 violations.
- Feature branches only. No commit to main. No `rm`. No remote branch deletion. No force push.

## Severity

| Emoji | Severity | Meaning |
| ----- | -------- | ------- |
| 🔴 | bug | wrong output, crash, safety hole, false hardware-validation claim |
| 🟡 | risk | edge case, missing guard, unverified claim, missing rebuild of an affected env |
| 🔵 | nit | style / naming |
| ❓ | question | needs author answer |

## Loop cap

- At most 4 builder/reviewer rounds.
- Then escalate to the human with the open findings.
- Reviewer escalates at iteration N >= 4 (`ESCALATE to human`).

## Honesty

- Nothing in this repo is validated on real hardware.
- Native tests and Wokwi do not validate `balance`.
- Sim-only programs `ttl_servo`, `stepper_motor`, `wokwi_ttl_servo` stay sim-only.
- These agents and rules are text, untested until smoke test #126.
- Hooks that enforce some rules come later.
