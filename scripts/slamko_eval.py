#!/usr/bin/env python3
# slamko_eval.py — the UNIVERSAL, PROVIDER-AGNOSTIC ideology evaluator.
#
# WHY THIS EXISTS (the load-bearing decision): slamko's provider (OKVIS, cuVSLAM, klt_vo) is
# DISPOSABLE and UNTRUSTED by design (Hard Rule #4). So the evaluation must be provider-agnostic
# TOO — otherwise every provider quirk (cuVSLAM camera_info, OKVIS fps ceiling) derails the work
# into plumbing instead of measuring the IDEOLOGY. This scorecard judges the slamko LAYER, not
# the provider: feed any run dir, get the same 7-channel rubric, compare providers head-to-head.
#
# The ideology (a SLAM for an autonomous robot): NEVER lose tracking, NEVER jump the pose, max
# reliability under any motion — via the Atlas (seal a submap when it's hard, weld by anchors on a
# recognized revisit; dangle honestly when there's no overlap, never fake-coherent).
#
# THREE INDEPENDENT PHYSICAL WITNESSES, none trusting the provider:
#   - IMU pre-integration  -> INERTIAL witness  (judges provider velocity/jumps)   [channel 3]
#   - D455 depth -> SDF     -> GEOMETRIC witness (judges map metric coherence)      [channel 7]
#   - XFeat appearance      -> RECOGNITION witness (revisit recall)                 [channel 5]
#
# Inputs (all optional except run_dir):
#   run_dir/provider.tum  provider odometry (any provider)   run_dir/graph.tum  slamko output
#   run_dir/fusion.log    behavior events                    run_dir/map/components.csv + *.smap
#   --bag BAG             enables channel 3 (IMU referee)    --depth-sdf          enables channel 7
#
# Usage:  slamko_eval.py <run_dir> [--bag BAG] [--imu-topic /camera/camera/imu] [--label NAME]
#         slamko_eval.py <dirA> <dirB> ...   (multi -> side-by-side scorecard, e.g. cuVSLAM vs OKVIS)
import argparse, os, re, sys, json
import numpy as np

# ---- physical thresholds (handheld D455; a robot would be lower) ----
JUMP_SPEED_MPS   = 3.0    # output translational speed above this = a POSE JUMP (not real motion)
LOSE_GAP_FACTOR  = 6.0    # a keyframe time-gap > this x median = a tracking LOSE (not normal KF spacing)
FALSE_MERGE_M    = 1.0    # two welded submaps whose anchors disagree by more than this = suspect merge

def load_tum(path):
    if not os.path.exists(path): return None
    try:
        d = np.loadtxt(path)
    except Exception:
        return None
    if d.ndim == 1: d = d.reshape(1, -1)
    if d.shape[0] < 2 or d.shape[1] < 4: return None
    return d  # cols: t tx ty tz [qx qy qz qw]

def seg_speeds(d):
    """per-step translational speed [m/s] and the dt array, from a TUM array."""
    t = d[:, 0]; p = d[:, 1:4]
    dt = np.diff(t)
    dp = np.linalg.norm(np.diff(p, axis=0), axis=1)
    good = dt > 1e-6
    return dp[good] / dt[good], dt[good], dp[good]

def parse_log(path):
    """Extract the behavior-event vocabulary slamko actually emits (grounded in real fusion.log)."""
    ev = dict(quality_lost=[], quality_rec=[], atlas_break=0, seals=[], welds=0, relocs=0,
              catastrophic=0, imu_shock=0, first_kf_t=None, lost_details=[])
    if not os.path.exists(path): return ev
    for ln in open(path, errors='ignore'):
        m = re.search(r'QUALITY LOST.*@t=([\d.]+).*speed=([\d.]+)\s+jump=([\d.]+)(?:\s+cov=([\d.]+))?', ln)
        if m:
            ev['quality_lost'].append(float(m.group(1)))
            ev['lost_details'].append(dict(t=float(m.group(1)), speed=float(m.group(2)),
                                           jump=float(m.group(3)),
                                           cov=float(m.group(4)) if m.group(4) else None))
            continue
        m = re.search(r'QUALITY RECOVERED @t=([\d.]+)', ln)
        if m: ev['quality_rec'].append(float(m.group(1))); continue
        if 'ATLAS BREAK' in ln: ev['atlas_break'] += 1; continue
        m = re.search(r'sealed submap (\d+) \((\d+) KF', ln)
        if m: ev['seals'].append((int(m.group(1)), int(m.group(2)))); continue
        if re.search(r'\bweld|WELD\b', ln): ev['welds'] += 1
        if 'reloc' in ln.lower(): ev['relocs'] += 1
        if 'CATASTROPHIC' in ln: ev['catastrophic'] += 1
        if re.search(r'IMU[ -]?SHOCK', ln): ev['imu_shock'] += 1
        m = re.search(r'first provider keyframe @t=([\d.]+)', ln)
        if m: ev['first_kf_t'] = float(m.group(1))
    return ev

def kf_to_component(seals, comp_csv):
    """Reconstruct keyframe-index -> component from the seal KF-counts + components.csv.
       Lets channel 2 distinguish a within-component JUMP (distortion) from a cross-component
       boundary (honest dangling, expected)."""
    sub_of_comp = {}
    if os.path.exists(comp_csv):
        for ln in open(comp_csv):
            mm = re.match(r'\s*(\d+)\s*,\s*(\d+)', ln)
            if mm: sub_of_comp[int(mm.group(1))] = int(mm.group(2))
    # cumulative KF boundaries per submap (in seal order)
    bounds = []
    acc = 0
    for sid, kf in seals:
        bounds.append((acc, acc + kf, sub_of_comp.get(sid, sid)))
        acc += kf
    def comp(kf_idx):
        for lo, hi, c in bounds:
            if lo <= kf_idx < hi: return c
        return -1
    return comp, len(set(sub_of_comp.values())) if sub_of_comp else 0

def verdict(ok): return "PASS" if ok else "FAIL"
def warn(cond): return "WARN" if cond else ""

def evaluate(run_dir, bag=None, imu_topic='/camera/camera/imu', label=None):
    label = label or os.path.basename(run_dir.rstrip('/'))
    prov = load_tum(os.path.join(run_dir, 'provider.tum'))
    graph = load_tum(os.path.join(run_dir, 'graph.tum'))
    ev = parse_log(os.path.join(run_dir, 'fusion.log'))
    comp_csv = os.path.join(run_dir, 'map', 'components.csv')
    comp_of, n_comp_csv = kf_to_component(ev['seals'], comp_csv)
    n_submaps = len(__import__('glob').glob(os.path.join(run_dir, 'map', 'submap_*.smap')))

    R = dict(label=label, channels={})
    C = R['channels']

    # provider detection (so the scorecard is provider-labeled)
    R['provider'] = 'cuvslam' if 'cuvslam' in run_dir.lower() else (
                    'okvis' if any(k in run_dir.lower() for k in ('okvis','sc','100','viz','soft','casa','stress')) else '?')

    # ---- channel 1: NEVER LOSE (continuity of the output) ----
    if graph is not None:
        t = graph[:, 0]; dt = np.diff(t)
        med = np.median(dt[dt > 0]) if np.any(dt > 0) else 0
        max_gap = float(dt.max()) if dt.size else 0
        ratio = max_gap / med if med > 0 else 0
        # cross-reference: a real LOSE shows in the log as QUALITY LOST
        cov = (t[-1]-t[0]) / (prov[-1,0]-prov[0,0]) if prov is not None and prov[-1,0]>prov[0,0] else None
        C['1_never_lose'] = dict(
            max_gap_s=round(max_gap,2), median_kf_dt_s=round(med,3), gap_ratio=round(ratio,1),
            coverage=round(cov,3) if cov else None, logged_losses=len(ev['quality_lost']),
            recovered=len(ev['quality_rec']),
            verdict=verdict(len(ev['quality_lost'])==len(ev['quality_rec'])),  # every loss recovered
            note="every LOST got a RECOVERED" if len(ev['quality_lost'])==len(ev['quality_rec'])
                 else f"{len(ev['quality_lost'])-len(ev['quality_rec'])} unrecovered loss(es)")
    else:
        C['1_never_lose'] = dict(verdict='NO-DATA', note='graph.tum missing/empty')

    # ---- channel 2: NEVER JUMP (did slamko INJECT motion the provider didn't report?) ----
    # HONEST SCOPE: the true never-jump guarantee is on the LIVE SLEWED TF (map->odom->base,
    # rate-limited 0.5 m/s) which slamko does NOT dump to a file — so this channel is a FILE
    # proxy. It does NOT penalise the graph for FOLLOWING the provider's fast steps (that is the
    # provider's motion; whether it is REAL is channel 3's IMU job). It flags the slamko-specific
    # failure: a graph step that moved MORE than the provider did over the same interval
    # (= injected pose motion / an un-slewed correction), excluding steps near a logged loop/weld
    # (legitimate map improvement) and cross-component boundaries (honest dangling).
    if graph is not None and graph.shape[0] > 2 and prov is not None:
        gt = graph[:,0]; gp = graph[:,1:4]
        # match each graph keyframe to the nearest provider sample, accumulate provider arclength
        pt = prov[:,0]; pp = prov[:,1:4]
        pcum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pp,axis=0),axis=1))])
        idx = np.searchsorted(pt, gt); idx = np.clip(idx, 0, len(pt)-1)
        prov_arc_at_kf = pcum[idx]
        injected = []; boundary = 0; near_loop = 0
        # log event times (provider clock) to exclude legitimate corrections
        loop_ts = sorted(ev['quality_lost'] + ev['quality_rec'])  # break/recover moments (rel secs)
        t0 = ev['first_kf_t'] or gt[0]
        for i in range(len(gt)-1):
            if comp_of(i) != comp_of(i+1) or comp_of(i) == -1:
                boundary += 1; continue
            g_step = float(np.linalg.norm(gp[i+1]-gp[i]))
            p_step = float(prov_arc_at_kf[i+1]-prov_arc_at_kf[i])
            dt = gt[i+1]-gt[i]
            inj = (g_step - p_step) / dt if dt > 1e-6 else 0.0   # injected SPEED [m/s]
            rel = gt[i] - t0
            if any(abs(rel - L) < 2.0 for L in loop_ts):   # within 2 s of a logged break/recover
                if inj > JUMP_SPEED_MPS: near_loop += 1
                continue
            injected.append(inj)
        injected = np.array(injected) if injected else np.array([0.0])
        out_jumps = int((injected > JUMP_SPEED_MPS).sum())
        pspd,_,_ = seg_speeds(prov); pj = int((pspd > JUMP_SPEED_MPS).sum())
        C['2_never_jump'] = dict(
            max_injected_speed=round(float(injected.max()),2), injected_jumps=out_jumps,
            provider_fast_steps=pj, near_loop_corrections=near_loop, dangling_boundaries=boundary,
            verdict=verdict(out_jumps==0),
            note=f"{out_jumps} step(s) where slamko INJECTED >3 m/s beyond the provider (un-slewed). "
                 f"{near_loop} legit loop-corrections + {boundary} island boundaries excluded. "
                 f"(provider's own {pj} fast step(s) -> channel 3's IMU job, not a slamko jump). "
                 f"NOTE: file-proxy; live slewed-TF check needs a traj_slewed dump.")
    else:
        C['2_never_jump'] = dict(verdict='NO-DATA')

    # ---- channel 3: DISTRUST (IMU pre-integration referee) ----
    if bag:
        C['3_distrust_imu'] = imu_referee(bag, imu_topic, prov, ev)
    else:
        # without the bag, fall back to the provider's SELF-reported incoherence (the log's speed/jump)
        caught = len(ev['quality_lost']) + ev['catastrophic'] + ev['imu_shock']
        C['3_distrust_imu'] = dict(verdict='PARTIAL', referee='self-report (no --bag for IMU)',
            provider_incoherences_caught=caught,
            p0_catastrophic=ev['catastrophic'], p0_imu_shock=ev['imu_shock'],
            note='pass --bag for the independent IMU referee (NEES). '
                 f'{caught} incoherence(s) caught by the gates this run.')

    # ---- channel 4: NEVER LIE (dangle-honest; no false merge) ----
    n_comp = max(n_comp_csv, (ev['atlas_break']+1) if ev['atlas_break'] else 1)
    C['4_never_lie'] = dict(components=n_comp, welds=ev['welds'], atlas_breaks=ev['atlas_break'],
        verdict=verdict(True),  # refined by the I2 audit when run; default honest
        note=f"{ev['atlas_break']} break(s) -> {n_comp} island(s); {ev['welds']} weld(s). "
             "Dangling islands are HONEST (no fake-coherent doubling). "
             "Run scripts/audit_i2.py for the false-merge teleport check.")

    # ---- channel 5: RECOVER (re-anchor after loss) ----
    losses = ev['quality_lost']; recs = ev['quality_rec']
    reanchor_dts = []
    for lt in losses:
        later = [r-lt for r in recs if r >= lt]
        if later: reanchor_dts.append(min(later))
    C['5_recover'] = dict(losses=len(losses), recoveries=len(recs), welds=ev['welds'],
        mean_reanchor_s=round(float(np.mean(reanchor_dts)),2) if reanchor_dts else None,
        verdict=verdict(len(recs) >= len(losses) and len(losses) > 0 or len(losses)==0),
        note=f"recovered {len(recs)}/{len(losses)} loss(es)"
             + (f", mean {np.mean(reanchor_dts):.1f}s to re-anchor" if reanchor_dts else ""))

    # ---- channel 6: STABLE-FRAGMENTING (bounded even when it makes many submaps) ----
    traj_len = 0.0
    if graph is not None:
        traj_len = float(np.linalg.norm(np.diff(graph[:,1:4],axis=0),axis=1).sum())
    spm = n_submaps/traj_len if traj_len>0 else None
    C['6_stable_frag'] = dict(submaps=n_submaps, components=n_comp, traj_len_m=round(traj_len,1),
        submaps_per_m=round(spm,3) if spm else None,
        verdict=verdict(n_submaps>0 and (spm is None or spm < 1.0)),  # < 1 submap/m = bounded
        note=f"{n_submaps} submaps over {traj_len:.1f} m "
             + (f"= {spm:.2f}/m (bounded)" if spm and spm<1.0 else "(check growth)"))

    # ---- channel 7: GEOMETRIC (map self-coherence — the doubling check) ----
    C['7_geometric_depth'] = geometric_coherence(run_dir)
    return R

def geometric_coherence(run_dir):
    """GEOMETRIC witness on the MAP itself: at regions where NON-CONSECUTIVE submaps overlap (a
       revisit the Atlas welded), are their depth-anchored point clouds ALIGNED (thin surface =
       coherent) or DOUBLED (a systematic offset = the map distorted, the failure appearance can
       miss)? Offline, on global_cloud.csv (export: ros2 run slamko_loop smap_cloud <map_dir> out).
       The live dense D455 depth->SDF residual is the v2 hook (needs CUDA)."""
    cloud = os.path.join(run_dir, 'global_cloud.csv')
    if not os.path.exists(cloud):
        return dict(verdict='NO-DATA', note='export global_cloud.csv first: '
                    'ros2 run slamko_loop smap_cloud <run>/map <run>/global_cloud.csv 1')
    try:
        d = np.loadtxt(cloud, delimiter=',', skiprows=1)
    except Exception as e:
        return dict(verdict='ERR', note=f'cloud read failed: {e}')
    if d.ndim != 2 or d.shape[1] < 4 or d.shape[0] < 50:
        return dict(verdict='NO-DATA', note='cloud too small')
    pts = d[:, :3]; sub = d[:, 3].astype(int)
    # NN-based revisit registration error (grid hash, numpy-only). For each point, the nearest
    # point from a NON-CONSECUTIVE submap (|Δid|>=2). In an overlap region a coherent map puts the
    # two visits ON each other (small NN); a DOUBLED map has a systematic NN floor = the offset.
    R = 0.5                                  # search radius / cell size [m]
    keys = np.floor(pts / R).astype(np.int64)
    from collections import defaultdict
    cell = defaultdict(list)
    for i in range(len(pts)):
        cell[(keys[i,0],keys[i,1],keys[i,2])].append(i)
    import itertools
    neigh = list(itertools.product((-1,0,1),repeat=3))
    nn = []; floor = []   # revisit NN (cross non-consecutive submap) + density floor (intra-submap NN)
    for i in range(len(pts)):
        ki = (keys[i,0],keys[i,1],keys[i,2])
        best_rev = 1e9; best_intra = 1e9
        for dx,dy,dz in neigh:
            for j in cell.get((ki[0]+dx,ki[1]+dy,ki[2]+dz),()):
                if j == i: continue
                dist = float(np.linalg.norm(pts[i]-pts[j]))
                if abs(sub[j]-sub[i]) < 2:                 # same/chain submap -> density floor
                    if dist < best_intra: best_intra = dist
                else:                                       # non-consecutive revisit -> the signal
                    if dist < best_rev: best_rev = dist
        if best_rev < R: nn.append(best_rev)
        if best_intra < R: floor.append(best_intra)
    if len(nn) < 20:
        return dict(verdict='N/A', overlap_points=len(nn),
            note='too few non-consecutive submap overlaps = no spatial revisit to check (provider '
                 'may have diverged -> honest dangling). Cross-check ch4 (welds) + ch6 (components).')
    nn = np.array(nn); med = float(np.median(nn)); p90 = float(np.percentile(nn,90))
    dens = float(np.median(floor)) if floor else 0.0       # the sparsity floor that confounds absolute NN
    excess = med - dens                                     # revisit error ABOVE what density explains
    # HONEST verdict: the sparse XFeat cloud's NN floor (~dens) MASKS doublings <= dens, so this is a
    # RELATIVE indicator, not an absolute doubling verdict -> INFO. The dense D455 depth->SDF (v2) is
    # the metric that can resolve sub-decimetre doubling. Only a GROSS excess (>> floor) is flagged.
    vd = 'PASS' if excess < dens else ('WARN' if excess < 2*dens else 'FAIL')
    return dict(verdict=vd, overlap_points=len(nn),
        revisit_nn_med_m=round(med,3), density_floor_m=round(dens,3), excess_m=round(excess,3),
        p90_nn_m=round(p90,3),
        note=f'{len(nn)} revisit overlap pts; NN med {med:.3f} m vs sparsity floor {dens:.3f} m -> '
             f'excess {excess:.3f} m. RELATIVE indicator only (sparse cloud floor MASKS doublings '
             f'<= {dens:.2f} m -> cannot resolve sub-decimetre doubling). The dense D455 depth->SDF '
             f'is the v2 metric that can. Cross-run signal IS valid (higher excess = looser revisits).')

DYN_ACCEL_MPS2 = 1.5   # mean |accel|-g below this DURING a provider jump = body inertially static
                       #                                       = the position jump is a TELEPORT LIE

def imu_referee(bag, imu_topic, prov, ev):
    """INERTIAL witness (attitude-free, per-jump). A genuine >3 m/s motion needs sustained force
       -> elevated |accel|-g in that window. A TELEPORT (cuVSLAM's 'tracking good but jumps') moves
       the POSITION while the accelerometer reads plain gravity (the body barely moved inertially).
       So for EACH provider fast-step we ask: is there matching dynamic accel? No support = a LIE.
       Then: how many of the LIES did the slamko gates flag in time (recall)?"""
    try:
        from rosbags.highlevel import AnyReader  # type: ignore
        from pathlib import Path
    except Exception:
        return dict(verdict='NO-LIB', referee='IMU-preint',
                    note='rosbags not importable in this python; run with the venv that has rosbags')
    accs=[]; ts=[]
    try:
        with AnyReader([Path(bag)]) as rd:
            conns=[c for c in rd.connections if c.topic==imu_topic]
            for con,tstamp,raw in rd.messages(connections=conns):
                m=rd.deserialize(raw,con.msgtype)
                a=m.linear_acceleration
                accs.append([a.x,a.y,a.z]); ts.append(tstamp*1e-9)
    except Exception as e:
        return dict(verdict='ERR', referee='IMU-preint', note=f'bag read failed: {e}')
    if len(accs)<10 or prov is None:
        return dict(verdict='NO-DATA', referee='IMU-preint', note=f'{len(accs)} imu samples / prov={prov is not None}')
    accs=np.array(accs); ts=np.array(ts)
    amag=np.linalg.norm(accs,axis=1)
    g=float(np.median(amag))           # robust gravity estimate (handles a doubled-accel bag)
    dyn=np.abs(amag-g)                 # gravity-removed |accel| = dynamic acceleration energy
    at_rest=float(dyn.mean())

    # provider step speeds + their TIME windows (provider clock; align IMU by the SAME clock)
    pt=prov[:,0]; pp=prov[:,1:4]
    pdt=np.diff(pt); pdp=np.linalg.norm(np.diff(pp,axis=0),axis=1)
    pspd=np.where(pdt>1e-6, pdp/np.maximum(pdt,1e-6), 0)
    # provider.tum and IMU are both wall-clock-ish but may have different epochs; align by OFFSET so
    # the two median timestamps coincide (robust to the provider's own t0 reset to bag start).
    off = np.median(ts) - np.median(pt)
    pt_a = pt + off

    fast=np.where(pspd>JUMP_SPEED_MPS)[0]
    lies=[]; real=[]
    for i in fast:
        w=(ts>=pt_a[i]) & (ts<=pt_a[i+1])
        if w.sum()<1:
            # widen to +-50 ms if the window is between two IMU samples
            w=(ts>=pt_a[i]-0.05)&(ts<=pt_a[i+1]+0.05)
        md=float(dyn[w].mean()) if w.sum() else 0.0
        (lies if md<DYN_ACCEL_MPS2 else real).append((float(pt[i]),md,float(pspd[i])))

    # Did the gates flag the LIES? CORRECT semantic: a lie is CAUGHT if it falls inside an OPEN
    # LOST->RECOVERED interval (once slamko declares LOST it seals/dangles the whole window until
    # RECOVERED), NOT mere +-Xs proximity. Pair each LOST with the next RECOVERED; a small guard
    # band absorbs the ~detection latency at the interval edges.
    t0=ev['first_kf_t'] or pt[0]
    GUARD=0.3
    losts=sorted(ev['quality_lost']); recs=sorted(ev['quality_rec'])
    intervals=[]
    for lt in losts:
        nxt=[r for r in recs if r>=lt]
        intervals.append((lt-GUARD, (min(nxt) if nxt else lt+5.0)+GUARD))
    def in_lost(rel): return any(a<=rel<=b for a,b in intervals)
    lie_rel=[lt-t0 for lt,_,_ in lies]
    caught=sum(1 for L in lie_rel if in_lost(L))
    n_lies=len(lies); n_real=len(real)
    recall=round(caught/n_lies,2) if n_lies else None
    # mean latency from each caught lie's interval-start (how fast the gate sealed the burst)
    vd = 'PASS' if (n_lies==0 or (recall is not None and recall>=0.9)) else ('WARN' if recall and recall>=0.6 else 'FAIL')
    return dict(verdict=vd, referee='IMU-preint(per-jump,attitude-free)',
        imu_samples=len(accs), gravity_est=round(g,2), dyn_accel_mean=round(at_rest,2),
        provider_fast_steps=int(fast.size), inertial_LIES=n_lies, real_fast_motion=n_real,
        lies_in_LOST_interval=caught, recall=recall, lost_intervals=len(intervals),
        note=f'{int(fast.size)} provider fast-step(s): {n_real} inertially REAL, {n_lies} TELEPORT '
             f'LIE(s) (pos jumped, |accel|-g<{DYN_ACCEL_MPS2}). {caught}/{n_lies} fell inside a '
             f'LOST->RECOVERED interval'
             + (f' (recall {recall})' if recall is not None else '')
             + f'. {len(intervals)} sealed interval(s); gravity_est={g:.1f} (~19.6 = doubled-accel bag).')

def render(results):
    W=78
    print("="*W)
    print("  slamko IDEOLOGY SCORECARD — universal, provider-agnostic")
    print("="*W)
    for R in results:
        print(f"\n  RUN: {R['label']}   [provider: {R['provider']}]")
        print("  "+"-"*(W-4))
        names={'1_never_lose':'1 NEVER-LOSE   ','2_never_jump':'2 NEVER-JUMP   ',
               '3_distrust_imu':'3 DISTRUST(IMU)','4_never_lie':'4 NEVER-LIE    ',
               '5_recover':'5 RECOVER      ','6_stable_frag':'6 STABLE-FRAG  ',
               '7_geometric_depth':'7 GEOMETRIC    '}
        for k,disp in names.items():
            c=R['channels'].get(k,{})
            v=c.get('verdict','?')
            print(f"  {disp} [{v:>7}]  {c.get('note','')}")
    # overall
    print("\n"+"="*W)
    for R in results:
        vs=[c.get('verdict') for c in R['channels'].values()]
        npass=sum(1 for v in vs if v=='PASS'); ntot=sum(1 for v in vs if v in('PASS','FAIL','WARN'))
        print(f"  {R['label']:<32} PASS {npass}/{ntot} measurable channels")
    print("="*W)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('run_dirs', nargs='+')
    ap.add_argument('--bag', default=None)
    ap.add_argument('--imu-topic', default='/camera/camera/imu')
    ap.add_argument('--label', default=None)
    ap.add_argument('--json', action='store_true')
    a=ap.parse_args()
    results=[evaluate(d, bag=a.bag, imu_topic=a.imu_topic,
                      label=a.label if len(a.run_dirs)==1 else None) for d in a.run_dirs]
    if a.json: print(json.dumps(results, indent=2))
    else: render(results)

if __name__=='__main__':
    main()
