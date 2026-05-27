# slamko — Documentation Process (docs stay true to code + tests)

The goal the user asked for: **comprehensive documentation is produced when tests
pass, and corrected when commits change things.** This is the lightweight system
that makes that happen — a convention + one guard script. No heavy tooling.

## The three doc layers (per module)

Every `slamko_*` module carries exactly three docs (create on first real work):

1. **`CLAUDE.md`** — orientation: role, how it fits the system (links to root
   `../CLAUDE.md` + `../MASTER_PLAN.md`), the contracts it implements/consumes,
   build + test command, current status. Update when the **interface / build /
   contract** changes.
2. **`docs/ARCHITECTURE.md`** — how it works (the design, the math, key files).
   Update on **design** changes.
3. **`docs/STATUS.md`** — a **living, dated progress log** + the current metrics
   (ATE / FPS / test results). Append an entry on **every validated change**.

Plus, per module, a detailed plan `docs/PLAN_<phase>.md` produced in plan mode
when work starts (this is the "start me in the folder → plan mode → good plan"
workflow).

## The validated stamp

Every design doc (`ARCHITECTURE.md`, `MASTER_PLAN.md`, `PLAN_*.md`) starts with:

```
<!-- validated: <commit-sha> <YYYY-MM-DD> · tests: <result> -->
```

It records the commit + date the doc was last confirmed true, and what test run
backed it (e.g. `tests: 6/6 gtest, MH_01 ATE 0.054`). A reader (or agent) can
instantly tell if a doc is behind HEAD.

## The workflow rule (every session / agent follows this)

```
implement a change
   │
   ▼
run the module's tests / bench   (colcon test; slamko_bench ATE if relevant)
   │
   ├── RED  → fix; do NOT touch docs yet
   │
   └── GREEN
        │
        ▼
   update docs/STATUS.md  (dated entry: what changed + the numbers)
   update CLAUDE.md / ARCHITECTURE.md  IF interface/design changed
   bump the <!-- validated --> stamp on any design doc you changed
        │
        ▼
   commit code + docs TOGETHER   (one commit; never code without its STATUS entry)
```

So docs only assert what the tests confirmed (comprehensive-on-green), and any
later commit that changes behavior forces a STATUS entry + stamp bump (corrected-
on-change). The git history then carries the doc↔code↔test linkage.

## The guard: `scripts/check_doc_freshness.sh`

For each module it compares the latest commit touching `src|include` against the
latest commit touching `docs/STATUS.md`/`CLAUDE.md`, and checks the `validated`
stamp isn't behind HEAD for that module. It **flags** (CI-warn / pre-commit)
modules whose code moved ahead of their docs. Run it before a release, or wire it
as a pre-commit / a Claude Code `Stop` hook so stale docs surface automatically.

## Why this and not auto-generated docs

Auto-gen (Doxygen) documents *signatures*, not *why / what's validated / what's
known-broken*. The STATUS-log + validated-stamp captures the load-bearing
knowledge (what works, the ATE number, what was tried and reverted) that a future
cold-start session actually needs — the same value the klt_vo `docs/13` living
doc and the memory system already provide, now made a per-module convention.
