# Cavecrew rules

Text rules for repo cavecrew agents. Untested until smoke test #126 (#126 ran in part; see the last Honesty bullet).

## Agents

- `architect`: plans, files issues. Graph + GitHub tools. No code edits.
- `repo-cavecrew-builder`: haiku. 1-2 file edits (files touched, new files included). Read/Grep/Glob/Edit/Write + graph MCP read tools. No Bash.
- `repo-cavecrew-reviewer`: haiku. Read-only diff review. Input: `base: <ref>` label or pasted diff; anything else is ambiguous.
- `slop-hunter`: haiku. Read-only dead-code and doc-slop reporter. Bash intended only (enforcement unverified, #126): `code-review-graph dead-code`, `code-review-graph dead-code --json`, `code-review-graph architecture`, `code-review-graph large-functions`.
- For a pasted diff, `repo-cavecrew-reviewer` skips `detect_changes_tool` and `get_review_context_tool` entirely.
- Plugin agents `caveman:cavecrew-builder` and `caveman:cavecrew-reviewer` are separate agents. Project files do not extend them. CLAUDE.md only points to this file; the SubagentStart hook in `.claude/settings.json` is what injects it (a probe showed the hook injects the rules into both reviewers: `repo-cavecrew-reviewer` and `caveman:cavecrew-reviewer` each quoted the injected text; the builders are untested until smoke test #126, and the slop-hunter Bash gate is unverified (see the last Honesty bullet)).
- Plugin agent `caveman:cavecrew-investigator` is not in the SubagentStart matcher in `.claude/settings.json`, so it gets no injected rules, including the no-graph-update rule. Its own definition lives in the plugin, outside this repo; on the maintainer's machine it listed Bash (git log/grep/find by instruction), not re-checked by the repo. Widening the matcher is an owner edit (settings.json is protected).
- Older local agents `builder`, `reviewer`, `investigator` still exist and are separate.

## Defaults

- Builders `repo-cavecrew-builder` and the plugin `caveman:cavecrew-builder` edit 1-2 files per run (files touched, new files included); the older local `builder` has no such cap.
- Reviewer and slop-hunter are read-only on source files (Bash enforcement unverified, #126).
- Model `haiku` for `repo-cavecrew-builder`, `repo-cavecrew-reviewer`, `slop-hunter`. The older local `reviewer.md` is sonnet.
- Main session runs git, builds, tests and `graphify update .` after code changes; the older local `builder`, `reviewer` and `investigator` are instructed never to run graph updates; `repo-cavecrew-builder` and `repo-cavecrew-reviewer` have no Bash (the older local `builder.md` has Bash), `slop-hunter` has only exact-string Bash with no update form (this describes its `tools` line, not the effective permissions; see Honesty and #154), and the plugin `caveman:cavecrew-investigator` gets no injection (its body is outside the repo; on the maintainer's machine it had no such instruction, unverified here) (the project allow list permits `graphify update *` and `code-review-graph update *`; whether `tools:` Bash entries restrict a subagent is unverified, #126).
- slop-hunter Bash is meant to be exact strings only (enforcement unverified, #126); filtered runs (e.g. `--kind`, `--limit`, `--file-pattern`, `--path`, `--min-lines`) are run by the main session.
- Refusal/output tokens: `ambiguous. ask:`, `too-big. split:`, `needs-confirm. op:` (also for requests to edit protected or generated files), `ESCALATE to human`, `No issues.`, `No slop found.`

## Handover

- Builder: files changed with line ranges; affected firmware envs if a `lib/` file changed; acceptance commands for the main session to run; anything not verified.
- Main session: commands run with real output; not-run list; open questions.
- Never claim a test passed unless it was run.

## Gates

- Generated files: `lib/balance_core/balance_gains_generated.h`, `test/test_balance_parity/vectors_generated.inc`, `wokwi/chips/wokwi-ttl-servo.bg.h`. Not edited by agents; the main session or owner regenerates them (`make wokwi-bg` for the bg.h file; the Python exporter in the freecad-workspace repo for the gains header and vectors). The `protect-files.sh` hook blocks Edit/Write on them (tested for the main session; subagents untested until #126; Bash writes are not covered).
- `.claude/settings.json`, `.claude/settings.local.json`, `CLAUDE.md`: not edited by agents; the `protect-files.sh` hook blocks them (and the generated files and the hook script itself): verified for the main session; subagents expected, untested until #126. A linked issue that authorises such a change means the owner applies it outside Claude Code or removes the hook entry temporarily. `settings.json` edits are batched into one PR the owner reviews in full.
- The guard matches `Edit|MultiEdit|Write` (settings.json matcher); Bash writes and NotebookEdit are not covered (a speed bump, not a boundary). jq is a hard requirement for both hooks: without it the guard allows everything and the SubagentStart hook silently injects nothing. The path match is case-sensitive (a differently-cased name on a case-insensitive filesystem would slip through). If protect-files.sh fails to parse (for example CRLF line endings), bash exits 2 and blocks every Edit/Write until the settings.json wrapper (bash -n first, fails open with a stderr warning) is applied; .gitattributes keeps *.sh in LF. A clone that already has a CRLF protect-files.sh stays blocked after merge: repair it with `sed -i 's/\r$//' .claude/hooks/protect-files.sh` (keeps local edits); the wrapper only turns that block into a fail-open.
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

Applies to `repo-cavecrew-reviewer`; the plugin `caveman:cavecrew-reviewer` receives these rules too but keeps its own output format; the older `reviewer.md` keeps its own scale. (🔵 nit only when the caller asks for a thorough review.)

## Loop cap

- At most 4 builder/reviewer rounds.
- Caller MUST pass `iteration: N` to the reviewer; main session counts rounds itself (a missing iteration is treated as 1, which gives no cap).
- At round 4, if any 🔴 or 🟡 finding is still open, escalate to the human with the open findings; otherwise report normally.
- Reviewer: at N >= 4, if any 🔴 or 🟡 finding is still open, output `ESCALATE to human` followed by the open findings; otherwise report normally. (Same rule as the main session; the reviewer output token is `ESCALATE to human`.)

## Honesty

- Nothing in this repo is validated on real hardware.
- Native tests and Wokwi do not validate `balance`.
- Sim-only programs `ttl_servo`, `stepper_motor`, `wokwi_ttl_servo` stay sim-only.
- These agents and rules are text, untested until smoke test #126 (#126 ran in part; see the last Honesty bullet).
- The hooks (SubagentStart injection, protect-files guard) exist but are only partly tested; see smoke test #126.
- slop-hunter Bash enforcement is still unverified (#126, #154). Evidence as of 2026-10-21, pasted or self-reported, none of it settling enforcement: the owner's global permission settings were reported to be auto-approving (read from the owner's file; the repo cannot check them); in one owner session whose status bar showed `manual mode on` (which mode applied is unknown), one permission prompt appeared on the owner's screen for a non-listed `git show --stat HEAD` run by slop-hunter, whether the subagent gate or something else caused that prompt is unknown (mode, allow list or other); an agent or model reporting "no prompt" is not evidence, only the owner's screen is; the #126 smoke run reported `dead-code --kind Function` running with no visible prompt (self-reported, logged but not evidence) and had the agent run that filtered form itself, which belongs to the main session. Not tested: write or destructive commands through slop-hunter, and whether deny rules reach subagents. The 'untested until smoke test #126' lines elsewhere in this file predate these runs; #126 ran in part and did not cover the slop-hunter Bash gate, write commands or deny rules.
