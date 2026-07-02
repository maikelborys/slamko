#!/usr/bin/env python3
"""battery_report.py — aggregate a battery run (scripts/battery.sh) into ONE markdown
table: per bag, the 7 slamko_eval channel verdicts + the map/coherence counts.

usage: battery_report.py <battery_root> [--out docs/battery/BATTERY_<tag>.md]
The per-run inputs are <root>/<name>/eval.json (a list with one result: label,
provider, channels{k:{verdict,note}}) and <root>/<name>/counts.txt (key=value)."""
import argparse, json, os, sys

CH = ['1_never_lose', '2_never_jump', '3_distrust_imu', '4_never_lie',
      '5_recover', '6_stable_frag', '7_geometric_depth']
CH_SHORT = ['lose', 'jump', 'IMUref', 'lie', 'recov', 'frag', 'geom']
COUNTS = ['submaps', 'components', 'welds', 'atlas_breaks', 'crashes']


def load_run(d):
    row = {'name': os.path.basename(d.rstrip('/'))}
    try:
        res = json.load(open(os.path.join(d, 'eval.json')))[0]
        for k in CH:
            row[k] = res['channels'].get(k, {}).get('verdict', '—')
    except Exception:
        for k in CH:
            row[k] = 'ERR'
    try:
        for line in open(os.path.join(d, 'counts.txt')):
            if '=' in line:
                k, v = line.strip().split('=', 1)
                row[k] = v
    except Exception:
        pass
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    runs = sorted(d for d in (os.path.join(a.root, n) for n in os.listdir(a.root))
                  if os.path.isdir(d))
    rows = [load_run(d) for d in runs]
    if not rows:
        sys.exit(f'no run dirs under {a.root}')

    tag = os.path.basename(a.root.rstrip('/'))
    lines = [f'# Battery report — {tag}', '',
             f'Root: `{a.root}` · channels: slamko_eval 7-channel ideology scorecard', '']
    hdr = ['bag'] + CH_SHORT + COUNTS
    lines.append('| ' + ' | '.join(hdr) + ' |')
    lines.append('|' + '---|' * len(hdr))
    npass = nfail = 0
    for r in rows:
        cells = [r['name']]
        for k in CH:
            v = r.get(k, '—')
            npass += v == 'PASS'
            nfail += v == 'FAIL'
            cells.append({'PASS': '✅', 'FAIL': '❌', 'WARN': '⚠️'}.get(v, v))
        cells += [str(r.get(k, '?')) for k in COUNTS]
        lines.append('| ' + ' | '.join(cells) + ' |')
    lines += ['', f'**Totals: {npass} PASS · {nfail} FAIL across '
              f'{len(rows)} bags.** Crashes must be 0 everywhere; FAIL on any '
              'never-* channel blocks flipping the immortal defaults (T2).']
    md = '\n'.join(lines) + '\n'
    print(md)
    if a.out:
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        open(a.out, 'w').write(md)
        print(f'-> {a.out}', file=sys.stderr)


if __name__ == '__main__':
    main()
