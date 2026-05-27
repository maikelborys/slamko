#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Automated PASS/FAIL gate for a never-lost run — so correctness is asserted, not
# eyeballed. From the run log + dumps it checks:
#   1. the expected recovery events fired  (#SEAL, #WELD, ended OK),
#   2. the weld anchors are SANE            (no "100 m jump in a 5 m room" — the
#      false-relocalization signature the gate is supposed to reject),
#   3. applying the welds HELPS             (anchor-corrected Sim3-ATE < raw ATE),
#   4. the corrected ATE is below a bound.
# Exit 0 = pass, 1 = fail. Reuses plot_neverlost for the geometry.
#
# usage: check_neverlost.py --log run.log --gt GT.tum --est est.tum
#          --submaps est_lm.csv.submaps --pose-epoch est.tum.epoch
#          [--expect-seals N --expect-welds N --max-anchor-m 5 --max-ate-cm 60]
import argparse
import re
import sys

import numpy as np

from plot_neverlost import (associate, correct_poses, load_epoch, load_submaps,
                            load_tum, umeyama)


def ate_cm(te, xe, tg, xg):
    m, idx = associate(te, tg)
    if m.sum() < 10:
        return None
    s, R, t = umeyama(xe[m], xg[idx[m]])
    xa = (s * (R @ xe.T).T) + t
    return float(np.sqrt(((xa[m] - xg[idx[m]]) ** 2).sum(1).mean()) * 100.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--est", required=True)
    ap.add_argument("--submaps", required=True)
    ap.add_argument("--pose-epoch", required=True)
    ap.add_argument("--expect-seals", type=int, default=None)
    ap.add_argument("--expect-welds", type=int, default=None)
    ap.add_argument("--max-anchor-m", type=float, default=5.0,
                    help="reject a weld that jumps more than this (false-reloc guard)")
    ap.add_argument("--max-ate-cm", type=float, default=60.0)
    a = ap.parse_args()

    fails, checks = [], []

    def check(name, ok, detail=""):
        checks.append((name, ok, detail))
        if not ok:
            fails.append(name)

    log = open(a.log, encoding="utf-8", errors="ignore").read()
    n_seal = len(re.findall(r"\] SEAL submap", log))
    n_weld = len(re.findall(r"\] WELD to submap", log))
    states = re.findall(r"\[neverlost\] state \d+ → (\d+)", log)
    ended_ok = states and states[-1] == "0"

    # 1. recovery events
    if a.expect_seals is not None:
        check("seals", n_seal == a.expect_seals, f"{n_seal} (want {a.expect_seals})")
    else:
        check("seals>0", n_seal > 0, str(n_seal))
    if a.expect_welds is not None:
        check("welds", n_weld == a.expect_welds, f"{n_weld} (want {a.expect_welds})")
    else:
        check("welds>0", n_weld > 0, str(n_weld))
    check("ended OK", bool(ended_ok), states[-1] if states else "no state transitions")

    # 2. weld anchors sane (translation within room scale)
    submaps = load_submaps(a.submaps)
    worst = max((np.linalg.norm(t) for *_, t in submaps), default=0.0)
    check("anchors sane", worst <= a.max_anchor_m,
          f"max |t|={worst:.2f} m (limit {a.max_anchor_m})")
    check("#submaps == #seals+1", len(submaps) == n_seal + 1,
          f"{len(submaps)} submaps")

    # 3 & 4. corrected ATE beats raw and is below the bound
    tg, xg = load_tum(a.gt)
    te, xe = load_tum(a.est)
    ets, esid = load_epoch(a.pose_epoch)
    ate_raw = ate_cm(te, xe, tg, xg)
    ate_cor = ate_cm(te, correct_poses(te, xe, ets, esid, submaps), tg, xg)
    if ate_raw is None or ate_cor is None:
        check("ATE computable", False, "too few GT associations")
    else:
        check("welds improve ATE", ate_cor < ate_raw,
              f"corrected {ate_cor:.1f} cm < raw {ate_raw:.1f} cm")
        check("corrected ATE bound", ate_cor <= a.max_ate_cm,
              f"{ate_cor:.1f} cm (limit {a.max_ate_cm})")

    print("=== never-lost auto-check ===")
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:22s} {detail}")
    if fails:
        print(f"RESULT: FAIL ({len(fails)} check(s): {', '.join(fails)})")
        sys.exit(1)
    print("RESULT: PASS")


if __name__ == "__main__":
    main()
