# Plan 02 — persistent MapPoints, the IMPLEMENTATION (drift-tolerant data association)

## STATUS 2026-06-21 — Phase D SHIPPED (lifelong maturity: n_obs persists + compounds)
The PLVS/ORB-SLAM3 MapPoint maturity now SURVIVES a shutdown and COMPOUNDS across sessions —
the lifelong immortal map. `MapLandmark` gained `int n_obs` (default 1); the .smap codec bumped
to **SMP6** (additive trailing per-landmark n_obs block — SMP1–SMP5 still load, n_obs defaults
to 1, full back-compat). The destructor back-prop (Phase B) writes each kept landmark's store
maturity before save; Phase C's prior seed RESTORES it (`MapPointStore::add(..., n_obs)` /
`nObs(id)`) instead of resetting to 1. Unit test `test_submap_io.cpp::MaturityRoundTrip` (+ n_obs
in every round-trip `expectEqual`); slamko_core 8/0.
- **casa suave @0.5, 2 sessions of the same place (refine on both, S2 prior=S1):** S1 max n_obs
  **37** (mean 3.17, 1165 confirmed ≥4); S2 loaded the SMP6 prior, the store started populated
  (4051 pts, maturity restored), re-confirmed, and S2 max n_obs **65** (mean 3.88, 1617 ≥4) —
  confidence climbed ABOVE a single session. Without SMP6 S2 would restart near 37.
- **THE INITIATIVE IS COMPLETE: A (within-session dedup) + B (multi-view refine + confidence)
  + C (cross-session seed) + D (persist + compound) all SHIPPED, opt-in, trajectory-neutral.**
  Follow-on (not blocking): re-save the PRIOR submaps with the session's new confirmations (reuse
  the `mature_` re-save path) so a point re-confirmed in S2 also bumps the on-disk PRIOR, not just
  S2's store/output — full bidirectional compounding. Today S2's OWN output already compounds.

## STATUS 2026-06-21 — Phase C SHIPPED (cross-session: seed the store from the prior)
A 2nd session of the same place no longer DOUBLES the prior — it dedups/refines into it. At
prior load (when `mappoint_xsession` + a prior + `mappoint_assoc`), the store is seeded with the
prior map's MapPoints (prior-global position + L2-normalised descriptor; `next_landmark_id_`
bumped past the max prior id). Once the session cross-session-localizes (T_global_map_ set by the
relocalizer), its sealed landmarks land in the same prior-global frame and associate into the
PRIOR points — drift-tolerant where prior_occ_'s voxel test misses. Opt-in (default OFF).
`scripts/ab_phaseC.sh` (3 serial sessions) + the prior+revisit overlay.
- **casa suave @0.5, same-bag revisit:** ON seeds the store with **3999 prior MapPoints** (OFF
  starts ~0 → no cross-session dedup possible); the revisit relocalizes (kf 0, 108 inliers) and
  Phase C culls **6092** cross-session duplicates the voxel test missed → session-2 NEW landmarks
  **2240 → 968 (−57%)**. The overlay shows OFF adding an offset doubled layer, ON melting in.
- **Honest scope:** Phase C only fires AFTER relocalization succeeds — it kills cross-session
  DOUBLING (where reloc works), not the EuRoC DANGLING (where the match was too weak to reloc at
  all — that's the recall ceiling, a separate problem). Run-to-run nondeterminism remains (off 5
  vs on 7 submaps); the isolation is the seed (3999 vs 0) + the descriptor-cull counter (6092).
- **Next (Phase D):** serialize MapPoint ids + n_obs across sessions (.smap schema bump) so the
  consensus + maturity compound over many visits — the lifelong immortal map.

## STATUS 2026-06-21 — Phase B SHIPPED (multi-view refine + back-prop + confidence)
The PLVS "re-observe the same point" SECOND half: a revisit re-observation no longer just
dedups (Phase A) — it folds into the MapPoint's running-mean **consensus** (position +
descriptor, `MapPointStore::refine`) and at shutdown **back-propagates** the consensus into the
persisted submap landmarks (global → submap-local via the refreshed anchor). Each point also
carries `n_obs` = how many visits confirmed it (ORB-SLAM3/PLVS MapPoint maturity). Opt-in
`mappoint_refine` (requires `mappoint_assoc`; default OFF). Dumps `map/mappoints.csv`
(id,x,y,z,n_obs); `scripts/render_confidence.py` renders the map coloured by confidence + the
maturity histogram. Unit tests `slamko_loop/test/test_mappoint_store.cpp` (associate + running-mean
refine, 3/3 green; full slamko_loop suite 20/0).
- **casa brutal @0.5 VPR:** 8136 MapPoints, **50% multi-observed** (4053 refined), n_obs max=53
  mean 2.67; **back-propagated consensus into 8136 landmarks**.
- **Trajectory-neutral (same discriminator as Phase A):** PhaseA-vs-PhaseB provider diff 0.71 m
  (OKVIS nondeterminism on brutal), fused diff 0.64 m — fused diverges LESS than the raw provider
  Phase B can't touch → the back-prop moves only stored landmark geometry, never a graph factor.
- **Next (Phase C):** seed the store from the prior map at load so revisits associate into PRIOR
  MapPoints → drives the cross-session merge (kills the EuRoC dangling). Then Phase D serialize.

## STATUS 2026-06-21 — Phase A RE-VALIDATED on brutal, neutrality proven by discriminator
Re-ran the A/B on `CASA1_brutal1` @0.5 VPR=true (`scripts/ab_phaseA.sh` + `ab_phaseA_report.py`,
both committed). Result: **landmarks 21360 → 8948 (−58%)**, submaps 16 → 17 (≈ same). The
doubling collapse is visible in the side-by-side top-down (`/tmp/phaseA_ab.png`): baseline walls
are doubled/thick, Phase A walls are single.

**Neutrality — the clean discriminator (not the off-vs-on number).** The raw off-vs-on trajectory
diff is 1.0 m mean — but that is OKVIS NONDETERMINISM on the brutal bag (223 vs 245 `TRACKING
FAILURE: quality=0`, different OKVIS paths), NOT Phase A. Proof: Phase A NEVER touches the provider,
yet **provider** off-vs-on = 1.10 m mean while **fused** off-vs-on = 0.99 m mean — the fused
diverges LESS than the raw provider it can't influence. If Phase A moved the trajectory the fused
would diverge MORE than the provider; it diverges less (loops pull both runs to the same map). So
Phase A is neutral by-construction AND empirically. The fine 4.6 cm neutrality number lives on SUAVE
(deterministic enough); brutal can only show the doubling collapse, not byte-neutrality.

## STATUS 2026-06-20 — Phase A SHIPPED (opt-in, validated on casa)
`MapPointStore` (header-only, slamko_loop) + `sealSubmap` hook behind `mappoint_assoc` (default
OFF). Drift-tolerant cross-submap dedup by descriptor cosine + position radius. **Made provably
trajectory-NEUTRAL** (v2): descriptor-culls are kept SEPARATE from the cull backstop denominator
and still mark occ_ for kept submaps → the graph receives identical inputs → only the STORED map
shrinks. (v1 fed the backstop and moved the trajectory 0.9 m — reverted.)
- **casa brutal @0.5:** kept landmarks 20983→8253 (−61%), 11749 drift-dup culls.
- **casa suave @0.5:** kept landmarks 12464→**4527 (−64%)**, **8027** drift-dup culls; trajectory
  off-vs-on 4.6 cm mean / 17 cm max (≈ OKVIS run-to-run noise) — NEUTRAL; 6 loops vs 5 (reloc
  intact); map structure preserved (corridor walls still outlined), just dedup-sparse.
- **OKVIS nondeterminism caveat:** a single off-vs-on can't be byte-exact (OKVIS drops ~1800
  frames nondeterministically) — neutrality is by-construction + the 4.6 cm agreement. A clean
  empirical proof needs a DETERMINISTIC provider replay (future harness).
- **Open tuning:** radius 0.4 m / cos 0.82 is aggressive (8027 culls). Sweep for the
  conservative knee before defaulting ON. Phase B (multi-view refine) + C (cross-session) next.


Supersedes the design sketch in PLAN_PERSISTENT_MAPPOINTS_01.md (which holds the decision +
EuRoC findings). This doc is the concrete, code-anchored build plan the user approved with
"planifica e implementa". Principle (user): "como orbslam3 pero más fuerte, más potente y más
SIMPLE" + "result + stability, sin romperse" → small reversible steps, each proven with a map
PNG + an A/B, opt-in flags so nothing destabilizes the working loose-fusion.

## The root cause, precisely (from reading the seal code)
`provider_fusion_node::sealSubmap()` does cross-submap dedup by **voxel occupancy** (`occ_`, a
set of occupied voxel keys; `provider_fusion_node.cpp:1599-1612,1678-1679`). A revisit's
landmark is culled only if its **global voxel** is already occupied. When the revisit carries
VIO drift > `lm_dedup_voxel_` (0.15 m), its points land in DIFFERENT voxels → the cull MISSES →
the duplicates are kept → a 2nd offset submap = the doubling the user hates.

ORB-SLAM3 never relies on voxel coincidence: it re-associates the same physical point by
**descriptor + reprojection** (drift-tolerant), then FUSES. slamko already HAS the descriptors
at seal (`KfRec.lm_desc`, 64-D XFeat, representative per merged landmark) — it just throws the
cross-submap descriptor association away and uses geometry-only voxels.

## The fix = give slamko persistent MapPoint identity by DESCRIPTOR association
A global **MapPointStore**: each MapPoint = id + global position + representative XFeat
descriptor (+ obs count). At seal, every candidate landmark is associated against the store by
**position-neighborhood (generous radius, tolerates drift) AND descriptor cosine** — matched =
the SAME physical point (cull the duplicate / later refine), unmatched = a genuinely new point
(register). This is the occ_ cull, but drift-TOLERANT.

## Phases (each: build → A/B on a revisit bag → map PNG → STATUS + commit; all opt-in)
- **Phase A — drift-tolerant cross-submap association (THIS STEP).**
  `slamko_loop/include/slamko_loop/mappoint_store.hpp` (header-only: `associate(pos,desc)->id|-1`,
  `add`, voxel-hash spatial index). Hook in `sealSubmap` behind param `mappoint_assoc` (default
  OFF): in the per-landmark cull loop, additionally query the store — a descriptor match within
  `mappoint_assoc_radius` (0.4 m) at cosine ≥ `mappoint_assoc_cos` (0.82) CULLS the duplicate
  (drift-tolerant), else register a new MapPoint. Monotonic: only removes MORE duplicates than
  occ_ ever could, never adds. **Proof:** A/B mappoint_assoc on/off on a self-revisit bag —
  total landmarks ↓, submaps ≤, `render_map_png.py` shows the doubling collapse, ATE no regress.
- **Phase B — multi-view refine.** Store accumulates observations; refine MapPoint global pos
  (running/robust mean) and nudge the matched OLD submap landmark toward consensus → revisit
  makes the map MORE accurate, not just dedup'd.
- **Phase C — cross-session.** Seed the store from the prior map's landmarks at load; revisits
  associate into prior MapPoints → drives the cross-session merge that EuRoC Stage-2 showed is
  missing (the held candidates would become real fuses). Ties to the EuRoC bugs (recall, weld).
- **Phase D — serialize.** Persist MapPoints (id continuity + obs) across sessions; .smap schema
  bump. Lifelong immortal map.

Phase A alone kills the within-session doubling (the immediate visible pain) and is fully
reversible (flag off = today's behavior, byte-for-byte).

## Proof bag
`/mnt/data/bno_ab/CASA1_brutal1_Stereo60_RGB30_BNO_trim` @rate0.5 VPR=true (the established
revisit test; rate≤0.5 avoids OKVIS GPU-contention divergence). A/B: `mappoint_assoc:=false`
(baseline) vs `true`. Compare sealed-submap count, summed landmark count, and the top-down PNG.
