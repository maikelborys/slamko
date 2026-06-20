# Plan 02 — persistent MapPoints, the IMPLEMENTATION (drift-tolerant data association)

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
