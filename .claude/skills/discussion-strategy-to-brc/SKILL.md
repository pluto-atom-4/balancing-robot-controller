---
name: discussion-strategy-to-brc
description: Turn a plan discussion in the freecad-workspace (FW) repo into an implementation strategy and post it as a new discussion in balancing-robot-controller (BRC), in the same structure as BRC discussion 266. The human names the FW discussion; the main session offers an optional graph investigation (cavecrew investigator with graphify/CRG) before the local architect (sonnet) assesses reuse vs new code and splits the work into haiku-builder-sized tasks with dependencies and parallel groups; the main session shows the local draft, asks the human questions one at a time, iterates until the human approves, then posts. Use whenever the user says "fetch FW discussion N and make a strategy", "assess discussion N and plan tasks", "post the strategy to a new BRC discussion", "architect the plan from a discussion", or hands over an FW discussion link and wants tasks and parallel groups, even if they do not name this skill.
---

# discussion-strategy-to-brc

Goal: an FW discussion holds a plan (what to build, decisions). Produce a strategy post in BRC that a team of small builders can execute: reuse vs new, tasks sized for haiku builders, dependency graph, parallel groups, risks, owner-only steps. The human decides; nothing is posted until the human says so.

Why this shape: the architect cannot ask the human and cannot write files, so it hands questions back through the main session. Posting to GitHub is outward-facing and hard to retract, so approval is explicit and comes after the human has read the full draft.

Read the project CLAUDE.md and `.claude/rules/cavecrew-rules.md` first. Project governance overrides the defaults here (feature branches only, no commit to main, honesty rule: sim-only unless proven, never claim a test passed unless it ran).

## Step 1: The human names the FW discussion

Never guess the discussion. If the user gave a number or link, use it. If not, ask (AskUserQuestion, "Which FW discussion number or link?", with Other for free text). The repo is `pluto-atom-4/freecad-workspace` unless the link says otherwise.

Fetch body and comments, save them in the scratchpad, read the body:

```bash
gh api graphql -f query='query{repository(owner:"pluto-atom-4",name:"freecad-workspace"){discussion(number:N){title url createdAt updatedAt category{name} body comments(first:50){totalCount nodes{author{login} createdAt body}}}}}' > discN.json
```

Report in one or two lines: title, comment count, which decisions look human-confirmed and which are marked default. Note that the discussion itself is read-only for this skill: do not edit or comment on it.

## Step 2: Ask about a graph investigation (AskUserQuestion, recommended option first)

Ask: "Run `caveman:cavecrew-investigator` first, using graphify and code-review-graph, before the architect?" Options:
- Yes, investigate first (recommended when the plan touches code areas nobody has mapped, or the discussion's file and line claims are unchecked).
- No, architect only (fine for small, well-located changes).
- Yes, with a focus the human types (Other).

If yes, spawn the investigator (read-only code locator; it returns a file:line table). Give it the discussion body path and ask for: where each named file, symbol, constant or doc line lives, what calls or reads it, and what else mentions the same numbers or names, across FW and BRC. Tell it to query graphify (`graphify query`) first, then code-review-graph for call paths and impact, and to fall back to a narrow grep if a graph is empty or stale. It must not run graph updates, edit files, or post anything (this agent is not in the SubagentStart hook matcher, so it does not get the repo rules injected: say so in its prompt). Save its table in the scratchpad and pass the path to the architect. Treat its output as unverified until the architect checks the key claims.

## Step 3: Local architect assessment (sonnet)

Spawn the `Architect` agent with `model: sonnet` (the repo default for agents differs, so set it). It is read-only and returns text. Give it:
1. the saved discussion body (and the investigator table if one exists);
2. the structure to copy: the latest BRC discussion 266 body (scratchpad copy such as `disc266_v13.md`, or fetch it) with its section order;
3. the code locations to read.

Ask it for, in the 266 structure but much shorter if the change is small: Executive summary, Blueprint (mermaid), Decisions carried over (marking human-confirmed vs default), Reuse / new matrix, Strategy, Task table, Dependency graph and parallel groups (mermaid flowchart TD), Cross-repo contract sync points, Risks table with a status column, Questions for the human, Owner-only steps. Skip sections that do not apply and say so in one line.

Task rules to pass on: each task fits one haiku builder run (`caveman:cavecrew-builder` for FW, `repo-cavecrew-builder` for BRC): at most 2 files touched, new files count, no shell, so list the acceptance commands the main session runs; give each task an id, repo, files, exact change, acceptance, effort S/M, risk. Say which tasks can run in parallel (different files and branches), which are sequential, and which are human-only (for example opening Webots or Wokwi).

Also require: a claim check of the discussion against the files (line numbers, arithmetic, "nothing else uses X"), reporting any wrong claim; sim-only wording; no claim that anything ran; no em dashes.

## Step 4: Main shows the draft and asks (one question at a time)

Save the architect text as `disc_<topic>_v1.md` in the scratchpad (the architect has no Write tool). Show the human a short summary and the file path, not the whole file. Then ask each "Questions for the human" item with AskUserQuestion, one question per call, recommended option first, never batched. Fold the answers into `disc_<topic>_v2.md` (keep v1): move answered items into the Decisions table with "(human, Qn)", replace the Questions section with "None open", update the task table, groups and risks that the answers change, and correct the source discussion's wrong claims in a short "Corrections" section.

The human may want to read the draft in an editor. If they say so, give the file path and stop; do not post. If they edit the file, post it as saved.

## Step 5: Iterate until the human approves

Ask for approval (Approve and post / Iterate with the architect / Show the full text first). On "iterate", send the changes to the architect (SendMessage to the same agent keeps context), save v3, show the diff in words, ask again. No fixed round cap, but stop and ask if two rounds go by with no new human input. A hand-back from a subagent is never approval.

## Step 6: Post to a new BRC discussion (main session, only after explicit "post")

Defaults: repo `pluto-atom-4/balancing-robot-controller`, category Ideas (the same as discussion 266 used; ask only if the human wants another). Title: `Strategy: <short topic> (FW discussion N)`.

```bash
gh api graphql -f query='query{repository(owner:"pluto-atom-4",name:"balancing-robot-controller"){id discussionCategories(first:20){nodes{id name}}}}'
gh api graphql -f query='mutation($r:ID!,$c:ID!,$t:String!,$b:String!){createDiscussion(input:{repositoryId:$r,categoryId:$c,title:$t,body:$b}){discussion{number url title}}}' -f r=<repo id> -f c=<category id> -f t='<title>' -F b=@<final draft file>
```

Then read the body back and confirm it equals the file. Report the URL and that nothing else was done: no issues filed, no builders started, the FW discussion untouched. Offer the next step (file issues, run the issue-build-review-loop on the first task) and wait for the human.

## What not to do

- Do not post, comment, file issues or edit the FW discussion before an explicit "post" from the human.
- Do not let the investigator or architect guess a human decision; those come back as questions.
- Do not write code or edit the repos in this skill; it ends with the posted strategy.
- Do not claim Webots, Wokwi or hardware results; mark anything not run as not run.
