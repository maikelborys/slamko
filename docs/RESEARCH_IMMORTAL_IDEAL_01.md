# RESEARCH_IMMORTAL_IDEAL_01 — what an ideal immortal SLAM should be, and slamko's exact gap

<!-- authored 2026-06-29 · synthesis of a 4-agent study (1 codebase audit ✅ + 3 web-research,
the web agents hit server rate-limits so the external citations below are from domain knowledge,
not freshly re-verified URLs — treat system/author/year as reliable, re-fetch before quoting). -->

**Governing principle (user, 2026-06-29 — memory `slamko-immortal-seal-on-doubt`):** seal on ANY
metric-coherence doubt; start a new island ONLY when tracking is confidently good again; many small
stable islands beat one big map; revisits weld + align them; small coherent islands align cleanly.

This doc answers: *how would an IDEAL immortal SLAM realise that principle, what do humanoids /
legged robots do to never get lost, and what exactly must slamko build.*

## 1. The "fragment-on-doubt + re-align-on-reloc" pattern IS the established SOTA

The user's principle is not exotic — it is the convergent design of every robust lifelong system:

- **ORB-SLAM3 Atlas** (Campos et al., IEEE T-RO 2021). One *active* map + a set of *non-active* maps.
  On tracking loss it **does not dead-reckon — it spawns a NEW map** and keeps the old one. When it
  later relocalizes into an old map, it **merges**. This is *literally* "seal-on-loss + merge-on-reloc".
  slamko's Atlas/island design is the same idea.
- **Cartographer** (Hess et al., ICRA 2016). Local SLAM builds **submaps** that are sealed ("finished")
  after a bounded number of scans; global SLAM is a pose-graph of loop-closure constraints **between
  submaps**. Submaps are locally coherent by construction; all non-rigidity lives in the inter-submap
  graph.
- **RTAB-Map** (Labbé & Michaud, JFR 2019). Working Memory / Long-Term Memory: bounds the online graph,
  pages old nodes to LTM, retrieves them on appearance match → unbounded lifelong operation with a
  bounded real-time cost. The never-lost-by-memory-management pattern.
- **maplab / maplab 2.0** (Schneider 2018; Cramariuc 2022). Multi-session VI mapping, submap-based,
  robust pose-graph relaxation, explicit map merging across sessions.
- **Voxgraph** (Reijgwart et al., RA-L 2020). Submap-based volumetric (TSDF) mapping; a pose-graph
  between submaps deforms the global map on optimization while each submap stays internally rigid.
  This is the volumetric analogue of slamko's sparse islands.
- **GLIM** (Koide et al., 2024), **Kimera** (Rosinol et al., ICRA 2020): factor-graph back-ends with
  submap/keyframe structure and robust outlier handling.

**Takeaway:** seal-and-fragment is the proven backbone; the differentiator is *when* to seal and
*whether you stop trusting the uncertain stretch*. Most systems seal on size/loss; few explicitly
**refuse to map a degenerate stretch** (the user's HOLD). That refusal is slamko's chance to be
stricter-than-SOTA on the "never lie" axis.

### 1b. SOTA corrections + the named pattern (primary sources — from the completed research)

- **The pattern has a NAME and an origin: "Atlas".** ORBSLAM-Atlas (Elvira, Tardós, Montiel, IROS 2019,
  arXiv:1908.11585) — "an unlimited number of disconnected sub-maps" + "a robust map merging algorithm".
  ORB-SLAM3 (arXiv:2007.11898) abstract, verbatim: *"when it gets lost, it starts a new map that will be
  seamlessly merged with previous maps when revisiting mapped areas."* **This IS the user's principle**,
  and it is the direct ancestor of slamko's disjoint-island / weld-on-match model.
- **CORRECTION to §3 below — sealed submaps are RE-POSED RIGIDLY, never internally deformed.** Every
  modern submap mapper (Voxgraph arXiv:2004.13154; GLIM arXiv:2407.10344; OKVIS2/DigiForest
  arXiv:2403.02280) treats a sealed submap as a **rigid body** and optimizes only the **global pose-graph
  over submap poses** (Sim3 when scale drifts); internal consistency is refined *before* sealing. **slamko
  already does this** (landmarks fixed per submap at seal, anchored to one node; the pose-graph moves the
  anchors). So the earlier "option C: add a rigid weld" is MISFRAMED — a soft pose-graph over rigid
  submaps IS the rigid treatment; the real refinement is *internal-consistency-before-seal* + an
  *overlap-based seal trigger* (GLIM seals at ~15 frames OR <5% overlap; OKVIS2 at overlap<0.4).
- **Gauge / "small aligns cleanly" — grounded but with an honest flag.** VO/VIO drift grows
  **super-linearly with distance**; monocular needs a **7-DoF Sim3** correction (scale drift, Strasdat
  RSS 2010), reduced to **4-DoF with IMU** (Zhang & Scaramuzza RA-L 2018) — so a small VI submap has a
  well-defined local frame up to 4-DoF and aligns rigidly. FLAG: no single paper proves the composite
  "small ⇒ aligns cleanly" sentence, and **the submap-size threshold where it stops being rigid-enough is
  an explicit OPEN PROBLEM**. So seal-on-doubt is principled, but *how eagerly* is genuinely unsolved —
  tune it empirically.
- **Robust welding is mandatory (false merge = catastrophic).** A single false loop corrupts the map and
  "fools multiple robust back-ends" (Cadena 2016, arXiv:1606.05830) → run welds **precision ≫ recall**,
  protected by **PCM** (max-clique consistency, ICRA 2018) / **GNC** (70-80% outliers, arXiv:1909.08605)
  / DCS / Switchable Constraints. slamko already uses PCM-style consensus + has the I2 never-false-merge
  validation — keep it.
- **Recall ceiling, QUANTIFIED.** RTAB-Map: **39-54% recall at 100% precision** over 2 km; ORB-SLAM3:
  **30-40% recall for ~100% precision**. Merging is conditional on recall → **unrecognized islands dangle
  permanently**. This is the measured cost of "dangling > lying" — expect it, mitigate with viewpoint
  coverage + a fiducial absolute reference (idea D).

## 2. Why small coherent islands align cleanly (the user's second insight — grounded)

A long trajectory accumulates drift, so it is **not a rigid body**: re-basing it onto a recognised
place with one rigid SE3 leaves a path-growing residual → the rotation/doubling artefact we already
root-caused (`slamko-xsession-rotation-rootcause`). A **small** submap sealed before drift accumulates
**is** ~rigid → relocalization (PnP / scan-match) returns a clean 6-DOF transform, and the global
pose-graph only has to place rigid bodies. This is exactly why Cartographer/Voxgraph/VILENS keep
submaps small and push non-rigidity to the inter-submap graph. There is a **sweet spot**: too big →
internal drift (can't align); too small → too few features to relocalize. Submap size is the bias
(drift) / variance (relocalizability) knob.

## 3. What humanoids / legged robots do to never get lost

The robots that do varied jobs and stay localised share four habits — none of which is "keep one
perfect metric map":

1. **Multi-sensor factor graph with a proprioceptive fallback.** The reference is **VILENS** (Wisth,
   Camurri, Fallon — Oxford DRS, IEEE T-RO 2023): a fixed-lag smoother fusing **IMU + leg kinematics +
   LiDAR + visual** as **between-factors with adaptive covariance**. Leg/wheel odometry is *always
   available* and becomes the backbone when vision/LiDAR degrade — degradation is **covariance
   inflation, never an if(sensor_ok) branch** (identical to slamko Hard Rule #3). **DigiForest**
   (Oxford+ETH, 2024) extends this to multi-robot lifelong submap mapping. → This is the direct
   answer to the earlier RGBD/leg-odom question: add it as **another between-factor with its own
   covariance**, exactly as VILENS adds leg odometry.
2. **Relocalization against a prior / topological map.** **Boston Dynamics Spot Autowalk / GraphNav**:
   a robot is *taught* a route, stored as a **topological waypoint graph** (each waypoint locally
   consistent, with recorded fiducials/features); at run time it **relocalizes at waypoints** rather
   than trusting one global metric frame. A waypoint graph ≈ the user's islands. **ANYmal**
   (ANYbotics): leg odometry + LiDAR SLAM with relocalization.
3. **Graceful degradation, not blind dead-reckoning.** Covariance inflation on degraded sensing; the
   estimate widens but stays consistent.
4. **Behaviour-level recovery.** When confidence collapses, robust stacks **stop and relocalize**
   instead of integrating garbage — the operational form of the user's HOLD.

**Adopt for slamko:** (a) proprioceptive/secondary-odometry fallback as extra between-factors
(VILENS); (b) treat islands like Spot waypoints — locally consistent, relocalize between them;
(c) the HOLD = the "stop trusting, wait to re-localize" behaviour, in mapping rather than motion.

## 4. How to detect "not metrically coherent" online (the seal trigger)

Ranked by practicality for a LIVE, provider-agnostic, low-compute gate:

1. **Feature-starvation / low-parallax (cheapest, catches the corridor-wall case directly).** Few
   tracked features / low parallax ⇒ translation along the view is **unobservable** ⇒ any reported
   forward motion is dead-reckoning. slamko **already computes** the XFeat keypoint count per keyframe
   (today only logged) + the stereo landmark count.
2. **Inertial plausibility — the IMU referee (provider-independent, catches teleport-lies).** Position
   moves fast while `|‖accel‖ − g| ≈ 0` ⇒ physically impossible ⇒ a lie. This is slamko_eval channel 3,
   proven to catch all 5 lies offline on `/tmp/slamko_casa_g1`. Cheap (one accel-magnitude per IMU
   sample).
3. **Degeneracy factor** (Zhang, Kaess & Singh, ICRA 2016 "On Degeneracy of Optimization-based State
   Estimation Problems"): the smallest eigenvalue of the information/Hessian matrix flags an
   unobservable direction (the corridor axis). The principled detector, but needs the Hessian — the
   provider has it internally; slamko would approximate it from feature geometry.
4. **NEES / NIS chi-square consistency**: innovation vs predicted covariance; persistent over-bound =
   inconsistent/divergent. Needs a second reference or a tuned bound — heavier, more for offline audit.

**Design choice:** combine the two cheapest INDEPENDENT witnesses — **feature count (vision)** +
**IMU plausibility (inertial)** — exactly because they are independent of the provider's own
(untrustworthy) pose. A degenerate stretch trips when vision is starved AND/OR the inertial witness
says the motion is a lie.

## 5. slamko's exact gap (from the codebase audit) and the build

**Already exists (plumbing done, tested):** disjoint islands via per-component floating gauges
(`pose_graph.cpp:232-271`), seal+branch (`provider_fusion_node.cpp` `sealSubmap`, `pending_break_`→new
component @1151), robust **between-factor** welds (`addLoopEdge` @2028), cross-session between/prior
(@1900-1994), proximity-E, `connectedComponents` re-tag @760. A LOST→RECOVERED 3-state machine exists
(@1063-1116) — **but OFF by default** and it does not HOLD.

**Missing — the two genuine pieces:**

- **A. A live metric-coherence gate using INDEPENDENT witnesses.** Today the only live `incoherent`
  test (@1051) uses the provider's own implied speed/jump/cov — the very signals we call
  untrustworthy. Build: add a rolling **accel buffer in `onImu`** (today it integrates only gyro
  @937), maintain robust `g` + `dyn=|‖a‖−g|`, and OR a `teleport_lie = fast_step && dyn_mean<~1.5`
  term plus a `feature_starved = keypoints<K || landmarks<L` term (the keypoint count is already
  computed @1534, only logged) into the DOUBT trigger. = porting slamko_eval channel 3 + a feature
  gate, LIVE.
- **B. A real HOLD state.** Today during LOST the node keeps feeding the chain, creating keyframes/
  nodes and buffering `pending_kfs_` (@1130-1145, 1633), and on RECOVER immediately starts a new
  island from dead-reckoned keyframes. Build: while `tracking_lost_`, **withhold all graph/map growth**
  (skip `chain_.feed`/`addKeyframe`/`pending_kfs_`); `gate_live_pose` keeps the *published* `odom→base`
  dead-reckoning smooth for Nav2, but **nothing enters the graph**; on `coherent_streak_ ≥ N` start a
  fresh island with no edge bridging the held stretch. Concentrated in `onOdometry`/`onKeyframe`.
- **C. (refined per §1b — NOT a rigid re-base)** slamko ALREADY treats submaps as rigid bodies welded
  by a soft pose-graph over anchors = the SOTA-correct model; don't add a one-shot rigid re-base. The
  real refinements: (i) refine each submap's internal consistency BEFORE sealing, (ii) add an
  **overlap-based seal trigger** (GLIM/OKVIS2: seal at <~5-40% overlap) alongside kf-count, (iii) keep
  welds **precision ≫ recall** with the existing PCM/consensus + I2 false-merge guard.

Default the whole thing ON (today `atlas_break_on_quality`/`atlas_break_on_loss` default OFF).

## 6. Honest trade-offs (carry these into the build)

- **Over-fragmentation / dangling islands.** Islands never revisited from a matching viewpoint dangle
  forever (the recall ceiling). Accepted: *dangling > lying*. Mitigate with viewpoint-aware reloc +
  proximity-E + multi-modal place recognition.
- **Submap-size sweet spot** (§2): seal too eagerly → islands too small to relocalize. The gate
  threshold must fire on *real* degeneracy (feature+inertial), not every minor dip.
- **Correlated witnesses.** A second visual odometry (RGBD) shares the camera with VIO → errors
  correlate; inflate its covariance (VILENS-style) so it doesn't double-count.

## 7b. Validated external references (first-hand, from the research agents that completed)

The humanoid/legged + IMU-witness agents ran the searches directly (their sub-agents had rate-limited);
these citations are first-hand. Strongest tier: Boston Dynamics SDK docs + Oxford DRS peer-reviewed line.

- **Spot GraphNav = the user's principle, productized.** "The world is a *locally consistent* graph of
  waypoints and edges… there is **no global frame**." Each waypoint stores a sensor snapshot (features,
  AprilTag detections); the robot localizes **relative to the nearest waypoint** and switches as it
  moves. AprilTag **fiducials** are the absolute re-init reference. Honest failure model: **"the robot
  will not attempt to recover from a lost condition on its own"** — an operator re-inits near a fiducial.
  (dev.bostondynamics.com/docs/concepts/autonomy/graphnav_tech_summary.html, …/graphnav_map_structure.html,
  …/initialization.html). This is *exactly* islands + per-island recognition + stop-when-lost.
- **Visual Teach & Repeat** (Furgale & Barfoot, JFR 2010): **32 km** route repetition via a **manifold of
  locally-consistent submaps**, no global frame — direct support for "small coherent islands > one big map".
- **VILENS** (Wisth/Camurri/Fallon, T-RO 2023, arXiv:2107.07243; precursor RA-L 2019 arXiv:1904.03048):
  leg odometry enters as a **preintegrated velocity factor with an ONLINE-ESTIMATED velocity-bias** term
  that is observable only through tight fusion — i.e. **don't trust raw secondary odometry, model its
  bias online**. 62% translational / 51% rotational error reduction over 2 h / 1.8 km on ANYmal. **This
  is the exact template for the RGBD/leg-odom-as-extra-factor question.**
- **Pronto** (Camurri/Fallon, Frontiers 2020): proprioceptive EKF core (IMU+leg) + loose LiDAR/visual
  corrections; ran on Atlas/Valkyrie/ANYmal. The "proprioceptive backbone always produces a pose" idea.
- **DigiForest** (arXiv:2506.20315, 2025): LIO=VILENS + factor-graph LiDAR SLAM + **proximity-gated
  (10–15 m) + ICP-verified** loop closure + multi-session relocalization against a prior map, fully
  GNSS-denied. slamko's proximity-E mirrors this recall-safe pattern.
- **IMU-as-witness literature** (the basis for the live coherence gate): **Forster et al.** preintegration
  residuals (ΔR, Δv, Δp) with covariance Σ, cost = Mahalanobis norm (T-RO 2017, arXiv:1512.02363) +
  **χ²/Mahalanobis gating** (Hesch RSS 2012; Jaekel IROS 2020) + **GLRT zero-velocity** detection (Skog
  T-BME 2010). HONEST FLAG: **no single paper canonizes the exact "position jumps while accel reads only
  gravity" teleport test** — it is the practical union of a preintegration-residual χ² gate + a
  ZUPT/gravity-consistency check. slamko's channel-3 referee is a legitimate, simple instance of it.
- **Consumer humanoids (Figure, Tesla Optimus, Apptronik, 1X, Unitree): no auditable localization
  architecture public** — "end-to-end learning" is marketing, not a never-lost design. Do NOT model
  slamko on their press. The verifiable never-lost engineering is Spot + ANYmal/DigiForest.

**Two genuinely new actionable ideas surfaced:**
- **(D) An optional AprilTag/fiducial absolute-reference between-factor for indoor deploys.** Every
  system that must *guarantee* re-init (Spot, warehouse Digit) uses fiducials as the cheap drift-bounding
  ground truth. A single tag factor gives slamko a hard anchor exactly where XFeat recall is
  viewpoint-limited (slamko's recurring limiter). Without an absolute reference, indoor drift is
  unbounded by information theory; the prior map + place recognition is the only other absolute ref.
- **(E) Add the secondary odometry (RGBD/leg) with an online-estimated bias (VILENS), not raw.** Inflate
  its covariance; let the bias be observed through fusion with the provider + XFeat. Avoids double-counting
  the camera-correlated error.

## 7. Recommended order

1. **Measure** what today's gates already cover: re-run `/tmp/slamko_casa_g1`'s bag with
   `gate_live_pose:=true atlas_break_on_quality:=true` + slewed dump → `slamko_eval`, recall 0/5 → ?/5.
2. **Build A** (live coherence gate: accel buffer + IMU-referee term + feature-starvation term).
3. **Build B** (HOLD state). Re-measure recall + map coherence (channels 2/3/7) + island count.
4. Optional **C** (rigid small-island weld) and the **VILENS-style 2nd-odometry between-factor**.
