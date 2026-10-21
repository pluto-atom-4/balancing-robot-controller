# git pull --ff-only: 'Cannot fast-forward to multiple branches'

Status: seen once on the maintainer machine (git 2.47.3). NOT reproduced. Cause UNKNOWN. Issue #147.

Local tooling note only. This is not a firmware doc.

## What happened

Right after `git checkout main`, with local `main` two commits behind `origin/main`,
`git pull -q --ff-only origin main` printed:

    fatal: Cannot fast-forward to multiple branches.

`git merge --ff-only origin/main` worked, so the remote history was fine.
Nothing was lost or changed.

Re-running the pull later, with `main` already current, printed "Already up to date.".
That run did not exercise the failing case.

## Config checked after the failure

- Fetch refspec `+refs/heads/*:refs/remotes/origin/*` (standard).
- `branch.main.remote = origin`, `branch.main.merge = refs/heads/main`.
- No `pull.*` settings.
- Global `core.hooksPath` points outside the repo (contents not inspected).
- A graphify hook runs a background rebuild on branch switch.
- An earlier plain `git fetch origin` ran in the same session.

## Hypotheses (BOTH UNTESTED)

(a) A post-checkout hook, or the background graphify rebuild, touched FETCH_HEAD
    or ran git at the same time as the pull.
(b) A stale multi-branch FETCH_HEAD left by an earlier plain `git fetch`.

## Workaround that worked

    git fetch origin && git merge --ff-only origin/main

## If it happens again

Before doing anything else, capture:

    cat .git/FETCH_HEAD
    GIT_TRACE=1 git pull --ff-only origin main

Attach the output to #147.

No change to this repo's hooks or settings was made for this. Hook changes live outside the repo.
