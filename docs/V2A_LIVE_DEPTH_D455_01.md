<!-- validated: live hardware 2026-06-23 (D455 SN 049122251369, FW 5.16.0.1) -->
# v2a groundwork — D455 live depth, validated on hardware

> Session 2026-06-23. Goal: prove the **live local depth** premise of v2a
> (D455 hardware depth → nvblox costmap, zero GPU, no HITNet contention) directly
> on the camera, at night and then in a lit room. Tooling:
> [`../scripts/d455_live/`](../scripts/d455_live/). Backend decision context:
> [`RESEARCH_LIFELONG_VOLUMETRIC_BACKEND_01.md`](RESEARCH_LIFELONG_VOLUMETRIC_BACKEND_01.md).

## What was validated (measured, not assumed)

| Claim | Result |
|---|---|
| D455 depth works **at night** | ✅ 94% coverage @ emitter on (active IR stereo — no visible light needed) |
| **60 fps** depth, 848×480 | ✅ 59.9 fps real, USB 3.2, zero GPU (on-ASIC stereo) |
| **90 fps** depth, 848×480 | ✅ 89.9 fps real |
| Range (indoor) | ~0.9–3.6 m typical, depth_scale 0.001 m/unit |
| Live → Rerun | ✅ smooth in the **native** viewer (web viewer chokes) |

## The emitter is the whole story (VIO ↔ depth conflict)

The D455 gives `infra1`+`infra2` (stereo pair) + `depth` + `color` **simultaneously
over one USB** — no splitter needed. The only knob that matters is the **IR emitter**:

| | depth | IR image (what VIO/reloc eats) |
|---|---|---|
| emitter **ON** | excellent (dots texture blank walls) | **dotted** — projected speckle; geometrically world-fixed so OKVIS VIO tracks it fine, but the pattern shifts with viewpoint → **hurts XFeat appearance reloc/loop recall** |
| emitter **OFF** | poor on textureless / at night | **clean** — real scene features, but **at night the frame is near-black** (brightness 20/255, ~100 corners) → OKVIS starves |

**Measured (night):** emitter off → brightness 20/255, 101 corners, depth cov 9%.
emitter on → brightness 62, 2000 corners (mostly dots), depth cov 94%.
**Measured (lit room):** emitter off → brightness **56**/255, clean usable VIO frame.
→ The "turn IR off for clean stereo" plan **works only with ambient/added light**.

## The right architecture: alternating emitter (`emitter_on_off`)

The D455 firmware toggles the emitter **per frame** (`RS2_OPTION_EMITTER_ON_OFF`,
**supported on this unit**). At 90 fps → **45 fps clean + 45 fps dotted**:

```
D455 @ 90fps, emitter_on_off=1
  ├─ 45fps CLEAN frames  → OKVIS (VIO) + XFeat (reloc)   [slamko provider]
  └─ 45fps DOTTED frames → nvblox (depth → costmap)      [v2a]
```
45 fps is plenty for both (OKVIS runs ~30–80, nvblox needs far less).

**Gotcha — metadata is offset by 1 frame.** `frame_emitter_mode` does NOT reliably
label the captured image (firmware reports commanded state; emitter toggles with
latency). **Fix that works:** classify each frame by **image content** — dot-energy
= fraction of high-pass pixels `(|ir − gauss(ir,σ=2)| > 12)`. Dotted frames score
high; clean low. **Median-split self-calibrates** (exact alternation → half high /
half low). `alternating_split_rerun.py` does this → rock-solid **45/45** live.

**Night robustness:** to actually use the clean channel in the dark, add a **constant
(unpatterned) IR floodlight** → the off-frame is illuminated-but-clean. Without it,
the "clean" frame IS the dark frame.

## Viewer lesson (load-bearing for any live viz)

- **Rerun web viewer (`--serve-web` + browser) chokes** on sustained live streams —
  freezes after seconds (proxy bandwidth). Symptom: "moves a bit then frozen",
  pressing play does nothing. NOT a stream bug (the producer kept emitting 45/45).
- **Native desktop viewer is smooth:** `DISPLAY=:0 rerun --port 9876` opens a wgpu
  window + gRPC server; scripts `connect_grpc` into it. Use this for live.
- Live uses the default `log_time` timeline (no custom `frame` timeline) → the
  viewer **auto-follows**; do **not** press "play" (replays from frame 0 = "frozen").
- Even native: throttle the PUSH (~12 Hz) + downsample cloud (~7k pts) while
  draining the camera at full rate — keeps latency low.

## Precision — honest framing (for the "build something huge" goal)

D455 stereo depth is **dense** (every pixel, denser than LiDAR's sparse beams) and
~**2% of range** (≈2 cm @ 1 m, ≈8 cm @ 4 m, error grows ~quadratically). So:
**excellent for indoor volumetric mapping (<5 m)**, NOT a LiDAR replacement at range
or outdoors. "Super-big" lifelong maps are a **storage/architecture** problem, not a
sensor one → the v2 split: nvblox local rolling costmap + **per-submap VDBFusion
tiles re-posed on loop closure, paged by region** (km-scale in single-digit GB).

## Next (v2a build)
Wire `depth_eye` (the 45 fps dotted depth) → **nvblox 5 m rolling** → 2D ESDF slice
→ Nav2 local costmap + live cubes in this same native Rerun. The live-depth pipe is
now proven; this is the remaining glue.
