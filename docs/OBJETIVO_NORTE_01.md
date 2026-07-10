<!-- validated: strategy consolidation 2026-07-10 · dirección de continuación AÚN NO DECIDIDA -->

# OBJETIVO NORTE — el framework "bastante robusto, no ideal" (y la bifurcación por plataforma)

**Palabras del usuario (2026-07-10):** el objetivo es un SLAM framework *bastante robusto
pero no ideal*, que sirva a los objetivos: SLAM inmortal que relocaliza, si hay revisita
se detecta la misma instancia, etc. **Y el principio de referencia: "si ORB-SLAM y
AirSLAM funcionan, el nuestro también debería."** Cómo seguimos: AÚN NO DEFINIDO — este
doc guarda las dos rutas listas para decidir.

## 1. El objetivo central (una frase)

Robot con mapa vitalicio que arranca → se localiza en su mapa anterior → reconoce
revisitas como la MISMA instancia → nunca se pierde ni miente → navega. 24/7 sin crashes.
**Barra "good enough": coherencia para navegar (10-20 cm local, decímetros global OK),
recall de reloc alto EN EL RÉGIMEN DE LA PLATAFORMA, cero falsos merges, recuperación
siempre.** No los 5 cm de paper.

## 2. El principio "si ellos pueden, nosotros podemos" — con la evidencia ya medida

| Sistema (en ESTE workspace) | Qué demuestra | Número |
|---|---|---|
| orbslam3_xfeat (GPL — solo referencia) | El MISMO XFeat 64-d pasa multi-sesión MH con la maquinaria adecuada (SearchAndFuse + welding-BA + asociación por proyección EN TRACKING) | 0.102 m MH01→03; V-chains sub-10 cm |
| AirSLAM_XFEAT | XFeat+LighterGlue con backend g2o intacto | 0.0386 m single-session |
| cuVSLAM nativo (nuestro fork) | El motor multi-sesión (save→localize→continue) alcanza la banda ORB en el par | 8.6 cm MH01→03 |
| slamko capa completa | Inmortal validado: never-lie, islas honestas, reloc casa, blackouts 100%, plateau +3%/visita | batería T1 55 PASS |

→ El descriptor NUNCA fue el problema (absuelto con datos). Lo que separa 1 m de 10 cm en
multi-sesión de VIEWPOINT CRUZADO es la **fusión de duplicados + asociación a nivel
tracking** (diagnóstico exhaustivo, capa a capa, en PLAN_CUVSLAM_MULTISESSION_01 §3b-§6).

## 3. La bifurcación por plataforma (decide el orden de los pasos)

**RÉGIMEN TERRESTRE (diferencial/4 ruedas):** revisita por los mismos caminos, misma
altura → el régimen donde TODO lo validado ya funciona. La fusión ORB-style probablemente
NO hace falta. Pasos (PLAN_ROBOT_DEPLOY_01):
1. Nav2 closed-loop en Gazebo (wheel-EKF + referee cinemático + gobernador de velocidad).
2. Multi-sesión en bags CASA (mismo viewpoint + depth) — predicción: 10-20 cm sin fusión.
   Si pasa, la fusión se desprioriza para esta rama.
3. KPI de servicio: recall de localización-al-arrancar sobre mapa previo (harness existe).
4. Robot real con kill-switch.

**RÉGIMEN DRON (si hay dron):** EuRoC ES el benchmark dron — las 2 semanas de diagnóstico
son la validación de este régimen. Cambios:
- **Fusión-a-la-creación vuelve al camino crítico** (multi-sesión aérea; el binding LC
  medido: 15 edges en UNA ventana → ~1 m; insertar en `lsi_grid.cpp` staged→activate).
- Sin ruedas: referee IMU-only + barómetro/rangefinder; **hover-on-LOST** con DR de vuelo
  (los gates pasan de robustez a safety con presupuesto ~1 s).
- Nav 3D real: el TSDF nvblox ya es 3D; cambia el planner (volumétrico o 2D por capas).
- Compute: Jetson Orin → cuVSLAM (diseñado para Jetson) aún más central; OKVIS justo.
- Anclas naturales multi-sesión: zonas de despegue/aterrizaje (como la plataforma MH).

**Tronco compartido (~80%):** provider+salud, seal-on-doubt/islas/honest-dangling, motor
multi-sesión, eval 7 canales, harness EuRoC completo — agnóstico por diseño (el contrato
slamko de día 1).

## 4. Estado de los activos al momento de decidir

- Fork cuVSLAM `slamko/trusted-health`: pnp_health + fix descriptores re-save + pase LC
  cross-session completo y medido (commits 9f861e0…110e506).
- Harness multi-sesión PyCuVSLAM (`cuvslam_multisession_euroc.py`, MS_SEQS, cadena de 5).
- Adapter slamko↔cuVSLAM P-A PASS; A/B providers cerrado (Inertial recomendado).
- Diagnóstico multi-sesión CERRADO con veredicto (PLAN_CUVSLAM_MULTISESSION_01 §6).
- Deploy plan terrestre (PLAN_ROBOT_DEPLOY_01) intacto y listo.

**Lo que NO haríamos en ninguna rama:** más EuRoC sin propósito, más capas LC, swap de
descriptor.
