# Cavecrew rules

Text rules for repo cavecrew agents. Untested until smoke test #126.

## Agents

- `architect`: plans, files issues. Graph + GitHub tools. No code edits.
- `repo-cavecrew-builder`: haiku. 1-2 file edits (files touched, new files included). Read/Grep/Glob/Edit/Write + graph MCP read tools. No Bash.
- `repo-cavecrew-reviewer`: haiku. Read-only diff review. Input: `base: <ref>` label or pasted diff; anything else is ambiguous.
- `slop-hunter`: haiku. Read-only dead-code and doc-slop reporter. Bash only: `code-review-graph dead-code`, `code-review-graph dead-code --json`, `code-review-graph architecture`, `code-review-graph large-functions`.
- For a pasted diff, `repo-cavecrew-reviewer` skips `detect_changes_tool` and `get_review_context_tool` entirely.
- Plugin agents `caveman:cavecrew-builder` and `caveman:cavecrew-reviewer` are separate agents. Project files do not extend them; these rules reach them only through CLAUDE.md and (later) hooks. This is the intended design; it is untested until smoke test #126.
- Older local agents `builder`, `reviewer`, `investigator` still exist and are separate.

## Defaults

- Builder edits 1-2 files per run (files touched, new files included).
- Reviewer and slop-hunter are read-only.
- Model `haiku` for `repo-cavecrew-builder`, `repo-cavecrew-reviewer`, `slop-hunter`. The older local `reviewer.md` is sonnet.
- Main session runs git, builds, tests. `repo-cavecrew-builder` has no shell (the older local `builder.md` has Bash).
- slop-hunter Bash is exact strings only; filtered runs (e.g. `--kind`, `--limit`, `--file-pattern`, `--path`, `--min-lines`) are run by the main session.
- Refusal/output tokens: `ambiguous. ask:`, `too-big. split:`, `needs-confirm. op:`, `ESCALATE to human`, `No issues.`, `No slop found.`

## Handover

- Builder: files changed with line ranges; affected firmware envs if a `lib/` file changed; acceptance commands for the main session to run; anything not verified.
- Main session: commands run with real output; not-run list; open questions.
- Never claim a test passed unless it was run.

## Gates

- Generated files, not edited: `lib/balance_core/balance_gains_generated.h`, `test/test_balance_parity/vectors_generated.inc`, `wokwi/chips/wokwi-ttl-servo.bg.h`. Edit only if the issue explicitly says so.
- `.claude/settings.json`, `.claude/settings.local.json`, `CLAUDE.md`: not edited unless the linked issue explicitly says so. Such `settings.json` edits are batched into one PR the owner reviews in full. (batching is a main-session duty; the agents only enforce the linked-issue gate)
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

Applies to `repo-cavecrew-reviewer` only (blue nit only when the caller asks for a thorough review); the older `reviewer.md` keeps its own scale.

## Loop cap

- At most 4 builder/reviewer rounds.
- Caller MUST pass `iteration: N` to the reviewer; main session counts rounds itself (a missing iteration is treated as 1, which gives no cap).
- Then escalate to the human with the open findings.
- Reviewer: at N >= 4, if any 🔴 or 🟡 finding is still open, output `ESCALATE to human` followed by the open findings; otherwise report normally.

## Honesty

- Nothing in this repo is validated on real hardware.
- Native tests and Wokwi do not validate `balance`.
- Sim-only programs `ttl_servo`, `stepper_motor`, `wokwi_ttl_servo` stay sim-only.
- These agents and rules are text, untested until smoke test #126.
- Hooks that enforce some rules come later.
