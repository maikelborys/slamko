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

## Palancas (en orden de apalancamiento)

1. **Welds distribuidos a lo largo del solape** — proximidad cross-session continua
   (relocalizeNear contra anchors del prior durante TODA la sesión, no solo VPR) +
   bajar el throttle de reloc en zona de prior. Arregla coherencia Y desbloquea el dedup.
2. Tras (1), re-medir B: el dedup cross-session debería acercarse al +3%/visita intra-sesión.
3. MH02/04 cuando el server ETH responda (`euroc_make_bag.py` listo).

GOTCHAS del harness: arg de launch VACÍO (`prior_map_dir:=`) = "malformed" y mata la sesión
entera en silencio; `loadSubMaps` exige `submaps.manifest` (la unión debe regenerarlo);
el engine TRT frío se come la primera sesión (cachear /tmp/slamko_vio_xfeat_752.engine antes).
Visual: `results/atlas_ms/atlas_multisession_3d.html`.
