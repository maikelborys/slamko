#!/usr/bin/env bash
# slamko doc-freshness guard. For each module, flag if source moved ahead of its
# docs (STATUS.md / CLAUDE.md). Run in CI / pre-commit / a Claude Code Stop hook.
# See docs/DOC_PROCESS.md. Exit 1 if any module is stale.
set -o pipefail
cd "$(dirname "$0")/.." || exit 2

stale=0
for mod in slamko_*/; do
  mod="${mod%/}"
  [ -d "$mod" ] || continue
  # latest commit touching source vs latest touching docs
  src_c=$(git log -1 --format=%h -- "$mod/src" "$mod/include" 2>/dev/null)
  doc_c=$(git log -1 --format=%h -- "$mod/docs/STATUS.md" "$mod/CLAUDE.md" 2>/dev/null)
  [ -z "$src_c" ] && continue          # no source yet (planned module) — skip
  if [ -z "$doc_c" ]; then
    echo "STALE  $mod: source exists (@$src_c) but no STATUS.md/CLAUDE.md doc"
    stale=1; continue
  fi
  # is the source commit an ancestor of the doc commit? if not, docs lag.
  if ! git merge-base --is-ancestor "$src_c" "$doc_c" 2>/dev/null; then
    echo "STALE  $mod: src @$src_c is ahead of docs @$doc_c — update docs/STATUS.md + bump validated stamp"
    stale=1
  else
    echo "ok     $mod (src @$src_c, docs @$doc_c)"
  fi
done

[ "$stale" -eq 0 ] && echo "all module docs fresh" || echo "!! stale docs above — see docs/DOC_PROCESS.md"
exit "$stale"
