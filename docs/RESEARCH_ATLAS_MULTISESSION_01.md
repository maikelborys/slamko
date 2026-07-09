<!-- validated: 2026-07-09 · runs results/atlas_ms (A) + atlas_ms_B (B) · harness commit 08049f5 -->

# RESEARCH — Atlas multi-sesión EuRoC MH (el test de validación de ORB-SLAM3)

**Pregunta del usuario:** procesar MH01→MH05 como sesiones separadas y medir si el multimap
las suelda en un mapa coherente + que los landmarks no crezcan sin techo.
**Ejecutado con MH01→MH03→MH05** (los datasets/bags MH02/04 no existen en disco y el server
ETH dio timeout — `scripts/euroc_make_bag.py` queda listo para añadirlos).
**Harness:** `scripts/atlas_multisession_euroc.sh` (encadena sesiones vía `prior_map_dir`
acumulado-por-UNIÓN + regenera `submaps.manifest`) + `atlas_multisession_report.py`.

## Resultados (A = dedup OFF · B = `mappoint_assoc+xsession` ON)

| métrica | A | B |
|---|---|---|
| MH01 own-Sim3 ATE | **3.4 cm** | 8.0 cm |
| MH03: factores X-SESSION | 58 (todos → submapa prior #1, al inicio) | similar |
| **MH03 coherencia en el frame fusionado** (Sim3 fijado SOLO con MH01) | **103 cm** | **98 cm** |
| MH05: matches cross-session | **0 → isla colgante honesta** | 0 |
| landmarks acumulados s1→s2→s3 | 3610→6685→**8176** (lineal) | 2256→4727→**5947** (−27%) |

## Lectura honesta

1. **La maquinaria Atlas FUNCIONA:** el prior carga (14→32→47 submapas, ids encadenados),
   MH03 suelda contra el mapa de MH01 desde el kf 0 (58 factores, 64–69 inliers), MH05 no
   matchea y queda **colgando sin weld falso** (never-lie ✓ — ORB-SLAM3 también la dejaría
   separada sin match de PR).
2. **PERO la coherencia métrica del mapa fusionado es ~1 m, no los 3–8 cm de ORB-SLAM3.**
   Causa raíz visible en el log: TODOS los welds anclan en UN solo sitio (el arranque,
   submapa #1). Un ancla local + drift libre por el resto del recorrido = 1 m. ORB-SLAM3
   fusiona con BA continuo sobre landmarks compartidos A LO LARGO de todo el solape;
   slamko (por diseño anchor-don't-weld) necesita **welds DISTRIBUIDOS** para lo mismo.
3. **Acotación de landmarks: la misma causa raíz.** B recorta 27% (el dedup intra-sesión
   ya baja s1 un 38%), pero s2 sigue añadiendo ~100% — el dedup cross-session
   (voxel 0.15 m) no puede disparar cuando las sesiones están desalineadas ~1 m entre sí.
   **Un solo defecto → dos síntomas** (coherencia y techo).
4. MH05 con 0 matches = el techo de recall por VIEWPOINT conocido (2026-06-19), reproducido
   cross-session en EuRoC.

## Run C (radio proximidad 8 m + dedup) + el análisis de raíz — 2026-07-09b

**Correcciones tras revisar el código (el usuario tenía razón):** el culling EXISTE en 3 capas
y está ON: (1) cull backstop a nivel SUBMAPA (`cull_enabled` default true, siembra
`prior_occ_` del prior, :1744/:2637); (2) dedup voxel al sellar; (3) `mappoint_assoc`
descriptor-tolerante. Y la proximidad cross-session (E) TAMBIÉN corre continua
(`relocalizeNear` vs anchors del prior, :2032). Nada de eso faltaba.

**Run C (PROX_RADIUS=8, MAPPOINT=on):** coherencia 90 cm (vs 103/98 A/B — marginal);
landmarks igual. **La distribución de welds lo explica todo:** kf 3-36 (despegue) Y
kf 857-868 (aterrizaje), TODOS contra el submapa prior #1 = la plataforma — el ÚNICO
lugar donde las trayectorias comparten viewpoint. El interior de la nave nunca verifica
(alturas/ángulos distintos) con radio 3 u 8. Dos regiones de anclaje al MISMO punto
físico → el medio cuelga → 90 cm. **Medición de co-localización de landmarks (A):**
s2-vs-s1 a <0.15 m = 7.7%, <1 m = 37.7%, mediana 3.08 m → ~⅔ de los landmarks de una
sesión con trayectoria distinta son puntos físicos NUEVOS (muestreo sparse dependiente
de viewpoint — XFeat es inocente; ORB/BRISK harían lo mismo). El cull backstop (70%)
no puede disparar con 8% de co-localización, y el viewpoint-aware correctamente PROTEGE
esos submapas (son el fix del dangling).

## La palanca real (diseño, próxima implementación)

**Asociación por PROYECCIÓN post-localización (el mecanismo de merge de ORB-SLAM3 que
slamko no tiene):** una vez `localized_` (T_global conocida), proyectar los landmarks 3D
del prior cercanos al frustum del KF actual EN la imagen (con la pose estimada), matchear
por descriptor con gate ancho en píxeles, PnP → weld edge. NO exige viewpoints de
apariencia similar (la proyección usa la GEOMETRÍA alineada) → welds distribuidos por
todo el solape → coherencia cm + el dedup existente por fin muerde. Es la versión sparse
del `depth_loop_refine` (que ya hace esto con geometría densa contra el ESDF cuando hay
depth). Encaja en anchor-don't-weld: más weld edges, cero BA cross-map, el grafo sigue
poses-only. Implementación: `XFeatRelocalizer::associateByProjection(query_kps,
prior_landmarks_in_frustum, T_estimate)` + los mismos gates I2 (inliers/PCM/Huber).

Después: re-medir C (coherencia debería caer a ~10-20 cm; dedup cross-session →
plateau) · MH02/04 cuando el server ETH responda (`euroc_make_bag.py` listo).

GOTCHAS del harness: arg de launch VACÍO (`prior_map_dir:=`) = "malformed" y mata la sesión
entera en silencio; `loadSubMaps` exige `submaps.manifest` (la unión debe regenerarlo);
el engine TRT frío se come la primera sesión (cachear /tmp/slamko_vio_xfeat_752.engine antes).
Visual: `results/atlas_ms/atlas_multisession_3d.html`.
