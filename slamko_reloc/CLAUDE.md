# slamko_reloc — relocalization plugins

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** given query features, recover pose against a feature/descriptor map.
Odometry-agnostic (DigiForest TreeLoc pattern). **Default: LiftFeat-m1** (the
*un-boosted* local descriptor — beats XFeat and is BoW-stable 3/3 merges; from the
user's `orbslam3_xfeat` `liftfeat-lightglue` branch). XFeat+LighterGlue = faster
fallback. DBoW2 place-recognition + "3 consecutive confirmations" gate.

**Implements:** `Relocalizer`. **Depends on:** slamko_core (+ libtorch/TRT for the
descriptor net). **Status:** planned. **Phase P3.** Apache/BSD only (no GPL).

**Starting cold here?** Read the 3 hub docs + this, then plan mode →
`docs/PLAN_P3_reloc.md`. Reuse extractor/matcher from `~/coding/orbslam3_xfeat`
(LiftFeat-m1) + `~/coding/AirSLAM_XFEAT` (XFeat TRT10).

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
