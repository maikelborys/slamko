#!/usr/bin/env python3
# Compare two TUM trajectories sample-by-sample at matching timestamps — the P-A
# gate metric (fused vs provider from the SAME run; NO alignment on purpose:
# Hard Rule #5, un-aligned divergence is the honest metric here).
import argparse
import sys

import numpy as np


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            v = line.split()
            if len(v) >= 8:
                rows.append([float(x) for x in v[:8]])
    return np.array(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ref')
    ap.add_argument('est')
    ap.add_argument('--tol', type=float, default=0.01, help='max position divergence [m]')
    a = ap.parse_args()

    ref, est = load(a.ref), load(a.est)
    if len(ref) == 0 or len(est) == 0:
        print(f'FAIL: empty trajectory (ref={len(ref)}, est={len(est)})')
        return 1
    # match by timestamp (both dumped per provider sample -> should be identical)
    tr = {round(t, 6): i for i, t in enumerate(ref[:, 0])}
    pairs = [(tr[round(t, 6)], j) for j, t in enumerate(est[:, 0]) if round(t, 6) in tr]
    if len(pairs) < 10:
        print(f'FAIL: only {len(pairs)} matching timestamps')
        return 1
    ri, ei = zip(*pairs)
    d = np.linalg.norm(ref[list(ri), 1:4] - est[list(ei), 1:4], axis=1)
    print(f'matched {len(pairs)}/{len(est)} samples  '
          f'max={d.max():.6f} m  mean={d.mean():.6f} m  rmse={np.sqrt((d**2).mean()):.6f} m')
    ok = d.max() <= a.tol
    print(f'{"PASS" if ok else "FAIL"} (max {d.max():.6f} m vs tol {a.tol} m)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
