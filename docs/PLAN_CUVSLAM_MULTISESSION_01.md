<!-- validated: measured 2026-07-10 (experiments in results/cuvslam_native_ms + 2 source studies) -->

# PLAN — multi-sesión a nivel ORB-SLAM3 con cuVSLAM-Slam como motor + slamko como supervisor

**Mandato del usuario:** "lo que haga falta, aunque refactoring total, pero con cuVSLAM +
slamko acercarnos a ORB-SLAM3 en el test multi-sesión EuRoC; mira cómo hace cuVSLAM la
localización con prior, los loops; tal vez reutilizamos directo; tal vez sus features en
vez de XFeat."

## 1. Lo medido (la base del plan)

**Experimento decisivo (`scripts/cuvslam_multisession_euroc.py`, PyCuVSLAM offline):**
cadena nativa `Slam` + `SaveMap` → `LocalizeInMap`(guess) → SEGUIR MAPEANDO:

| | slamko sparse (XFeat welds) | **cuVSLAM nativo** | ORB-SLAM3 ref |
|---|---|---|---|
| MH03 coherencia en frame MH01 | 90–103 cm | **8.6–10.2 cm** | — |
| JOINT Sim3 MH01+MH03 | ~59 cm | **5.4 cm** (escala 1.0009) | ~3–8 cm ✓ |
| MH05 | dangling (0 matches) | localiza en el frame ~600 con RETRY | (Atlas: mergea cuando PR dispara) |

GOTCHA medido: localizar con la imagen de t mientras el tail/odometría empezó en otro
origen MIS-BINDEA el grafo (MH05 3.2 m) → el flujo correcto es LIVE: trackear
continuamente e intentar localize con la imagen ACTUAL (fix aplicado; re-medición en curso).

**Estudio ORB-SLAM3 (orbslam3_xfeat, file:line en el informe):** la receta 1 m → cm es
(i) fusión de landmarks por PROYECCIÓN en la ventana del weld (`SearchAndFuse`, radio 4,
TH_LOW) + (ii) UN BA local de esa ventana con el prior FIJO; el essential-graph propaga;
el GBA es pulido NO load-bearing (ni siquiera se corre incondicionalmente). El culling de
KFs redundantes (≥90%/3-obs) es el techo de landmarks en revisitas.

**Estudio cuVSLAM slam/localizer (file:line en el informe):** tras `LocalizeInMap`, el
mapa cargado REEMPLAZA al vivo y la sesión CONTINÚA añadiéndole keyframes; los loop
closures disparan entre KFs nuevos y landmarks CARGADOS por TODO el solape (LSI espacial
+ guess de pose + NCC con warp afín en modo mapping) → muchos edges de unión → PGO denso
= su vía a los cm SIN BA de landmarks. **La unión por grafo de poses ERA suficiente; a
slamko le fallaba el recall del matcher, no el enfoque.**

## 2. La arquitectura (el reparto de papeles)

```
cuVSLAM (fork slamko/trusted-health)          slamko
──────────────────────────────────           ─────────────────────────────────
Odometry (pnp_health ya expuesto)      →     gates never-jump / referee / EKF ruedas
Slam: mapa métrico multi-sesión,             Atlas de MAPAS (islas = map-dirs cuVSLAM)
  LC denso vs landmarks cargados,            política seal-on-doubt / honest dangling
  PGO, LMDB persist, LocalizeInMap     ←     GUESS provider: XFeat/EigenPlaces VPR
slam_path (trayectoria retro-suavizada) →    TSDF/costmaps (bend con slam_path)
                                             eval 7-canales (provider sigue untrusted)
```

- **El guess es de slamko**: el sweep de cuVSLAM es yaw-only y NCC viewpoint-sensible —
  necesita un guess decente. XFeat/EigenPlaces (viewpoint-robusto, kidnapped-capable)
  PROPONE T_guess; `LocalizeInMap` VERIFICA y suelda métricamente. Cada uno en lo suyo.
- **Isla = un map-dir cuVSLAM**: si localize falla sostenido → seguir mapeando en mapa
  propio (dangling honesto) + reintentos periódicos con guesses de VPR; al lograrlo, la
  isla se re-basa (el mecanismo nativo swap+stitch). El modelo de islas de slamko intacto.
- **El stream vivo sigue siendo odometría gateada** (never-jump/slew) — el slam_path es
  la capa retro-suavizada para mapa/costmaps, nunca el pose stream de control.
- Double-loop-closure rule REFRAMED: dentro de UN mapa manda cuVSLAM-Slam; ENTRE mapas
  (islas/sesiones sin localizar) manda slamko. Sin doble optimización del mismo lazo.

## 3. Fases

**P1 — fork fix (REQUERIDO para cadenas 3+ sesiones): descriptor-loss en re-save.**
`SyncDatabase` solo escribe descriptor si `landmark.fd` está presente; los cargados
read-only tienen fd liberado + el DB fuente se reemplaza ANTES del sync
(`lsi_grid.cpp:135-155,436-439`) → un mapa re-guardado pierde descriptores de landmarks
del prior. Fix: materializar descriptores (`GetLandmarkFeatureDescriptor`) antes de
`AttachToNewDatabase` (patrón `SyncDatabaseReverse` `lsi_grid.cpp:213-235`). + añadir el
test upstream que falta: save→localize→continue→re-save→localize.

**P2 — adapter slamko:** extender `CuvslamProvider` con `Slam` (patrón Tracker en C++):
config `slam=true, map_dir, sync_mode`; servicios ROS `~/save_map`, `~/localize_in_map`
(guess de slamko-reloc o pose actual); publicar `~/slam_path`; política de islas +
reintentos en el nodo. slamko_loop conserva XFeat SOLO como guess/VPR (no welds métricos
sparse en el camino cuVSLAM).

**P3 — validación EuRoC completa:** cadena 5 sesiones (MH02/04 pendientes del server ETH;
`euroc_make_bag.py` listo — el experimento PyCuVSLAM no necesita bags, puede correr los 5
DATASETS ya con datasets descargados), tabla vs ORB-SLAM3, + acotación de landmarks del
mapa unido (LSI + `ReduceKeyframes`/`max_map_size` = su culling; medir crecimiento).

**P4 — casa/robot:** misma cadena con D455 (Inertial), guess de slamko-reloc, TSDF
harvest sin cambios. El test multi-sesión casa (same-viewpoint + depth) valida el sistema
completo.

## 3b. RESULTADOS de la cadena completa 5 sesiones (2026-07-10, con el fix P1)

**La cadena canónica MH01→02→03→04→05 CORRE entera**: cada sesión localiza (frame
300–600, flujo live con reintentos) y continúa mapeando; el fix de descriptores (fork
`c8ebed4`) preserva los priors en cada re-save. Coherencia post-localización bajo
T(MH01): MH01 4.0 · MH02 104 · MH03 67 · MH04 204 · MH05 333 cm → joint 105 cm.

**El par directo MH01→MH03 da 8.6 cm; la cadena degrada. Diagnóstico instrumentado:**
- Los LC internos SÍ corren durante la sesión 2 (Metrics: 73 good landmarks en el último
  evento) pero la coherencia no converge a cm → **el binding contra los landmarks
  CARGADOS tras el stitch es débil/ausente**; el 8.6 cm del par venía de UN buen stitch +
  drift propio bajo (~0.1% de 130 m), no de binding continuo.
- `max_map_size` descartado (220 nodos = decimación normal de KF, no poda; subirlo a 5000
  no cambia nada).
- El grafo (`get_pose_graph`) no expone timestamps y la frontera por IDs necesita más
  instrumentación (pendiente: un fprintf en `ApplyLoopClosureResult` slam.cpp:306 con los
  kf-ids → separa LC in-session vs vs-loaded en un run).

**Diagnóstico completado hasta el fondo (2026-07-10b, fork a80a083):**
1. `ApplyLoopClosureResult` instrumentado → **sesión 2: 21 LC edges, TODOS a keyframes
   propios, 0 a cargados.** Los landmarks frescos propios ECLIPSAN a los cargados en la
   selección — el problema exacto que ORB-SLAM3 resuelve con SearchAndFuse (fusión de
   duplicados). Sin fusión, el mapa cargado queda en sombra → cada sesión cuelga de su
   único stitch (~1 m).
2. **Fix en curso (plumbing completo, filtro aún no dispara):** segundo pase de
   `DetectLoopClosure` RESTRINGIDO a landmarks cargados (`LoopClosureTask.
   loaded_kf_boundary` + filtro en lcs_simple + boundary capturado en el swap de
   LocalizeInMap + pase en AsyncSlam con funnel). MEDIDO: `sel=0` — las RELACIONES de
   los landmarks cargados NO están en el LSI en memoria (viven en pose graph/DB), el
   predicado por relaciones rechaza todo.
3. **HECHO (fork 586401a): el pase restringido FUNCIONA de punta a punta** — set de
   landmarks cargados desde el pose graph + índice inverso `loaded_relations_` para que
   `FindKeyframeWithMostLandmarks` pueda anclar el edge a keyframes CARGADOS (el Apply
   fallaba en silencio con InvalidKeyFrameId). Funnel: sel=486-801, PnP-good=44-134,
   **10 binds aplicados** en MH01→MH02. Coherencia AÚN 103 cm: los binds son escasos y
   agrupados donde los viewpoints coinciden — la física de viewpoint en su última capa.
4. **SIGUIENTE PASO EXACTO (sesión fresca):**
   a. loguear kf/tiempo de cada bind → confirmar clustering; b. subir peso/probar
   covarianza de los x-edges en el PGO; c. si el clustering es el muro → **Plan B:
   fusión de duplicados a la creación** (asociar landmark nuevo → cargado en el LSI,
   estilo SearchAndFuse) — cambia el grafo de observaciones y hace que TODO LC propio
   sea también cross-session. d. El gate ≥40 de two_steps_easy si el pool se queda corto.
5. **VIEJO SIGUIENTE PASO (ya ejecutado):** cambiar la fuente del predicado a `map_.pose_graph_`
   (que sí carga las relaciones landmark→keyframe), o materializar las relaciones al
   cargar (el patrón del fix de descriptores c8ebed4). Después: re-medir MH01→MH02
   (objetivo: cross-binds > 0 distribuidos y coherencia 104 cm → ~10-20 cm), luego la
   cadena de 5, luego el gate ≥40 de two_steps_easy si el pool restringido se queda corto.
4. Plan B si el pase restringido no basta: fusión de duplicados al estilo SearchAndFuse
   (asociar landmark nuevo → cargado en el LSI al crear) — el refactor mayor autorizado.

## 4. Riesgos/gotchas

- Sweep yaw-only + NCC shift-only en el localizer (`localizer.cpp`, `kShiTomasi2`
  n_affine=0) → guess obligatorio (slamko lo da); pitch/roll grandes no localizables.
- Persistencia: el DB cargado es READ-ONLY → mapear continuo vive en RAM hasta save_map
  (política de auto-save del supervisor).
- Sin metadata de sesión/rig en el DB (gap upstream) — slamko debe guardar sidecar
  (rig fingerprint, fecha, frame tag) junto al map-dir.
- Timestamps hacia atrás en el stitch = el gotcha del 3.2 m (flujo live obligatorio).
- Los STDescriptors (parches 9×9/nivel, ~1.4 KB/landmark) NO sustituyen a XFeat para
  retrieval global (shift-only NCC); descartada la opción "features de cuVSLAM en vez de
  XFeat" para el guess — sí para el weld métrico (que es donde ya los usa).
