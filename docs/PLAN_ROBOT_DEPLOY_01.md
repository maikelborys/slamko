<!-- validated: design-only (no code) · 2026-07-09 · from the deploy-gap consulting session -->

# PLAN — sistema completo para el robot diferencial/4-ruedas (deploy gap analysis)

**Pregunta del usuario:** ¿qué falta para un sistema lifelong completo instalable en el
robot, que sepa dónde ir y con qué intensidad? **Estado percepción/mapa: ~60% hecho y
validado** (provider intercambiable + salud, lifelong seal/isla/weld, TSDF, dual costmaps,
eval). Lo que falta, por orden:

## 1. Wheel odometry + referee cinemático (el hueco #1) — Step 3 del plan original

- `robot_localization` EKF: `/odom_wheel` (diff_drive_controller) + provider odom →
  `/odometry/filtered` = `odom→base`; el `map→odom` de slamko intacto.
- **Referee cinemático** (más fuerte que el IMU referee): un diferencial no se mueve
  lateral (v_y≈0) ni supera v_ruedas. Gate por frame:
  `|v_provider - v_wheel| > k` o `|v_y| > eps` ⇒ SUSPECT (inflar cov, mismo camino que
  pnp_health). Slip: `cmd_vel` vs ruedas vs visual ⇒ rueda patinando / atascado.
- Validación: Gazebo (diff_bot_sim ya publica /odom_wheel) — tapar la "cámara" en sim y
  sobrevivir en dead-reckoning de ruedas; re-anclar al volver la visión.

## 2. Nav2 closed-loop (ya era el NEXT)

Planner global sobre `~/volumetric_costmap` (latched) + controller MPPI sobre
`~/local_costmap` (dinámico 45 Hz) + lifecycle gateado en `localized` + collision monitor.
**Recovery policy propia:** NUNCA el spin-recovery de Nav2 con SLAM en duda (corrompe el
mapa) — la recovery correcta es la de slamko: parar → sellar → isla → relocalizar.

## 3. Gobernador de velocidad por salud ("con qué intensidad") — nodo pequeño, alto valor

`v_max = v_nom · f(salud, entorno)`:
- salud OK + zona conocida (recall alto) → 1.0 · v_nom
- SUSPECT sostenido (pnp_health / referee ruedas / IMU) → 0.5 · v_nom
- LOST → 0 (parar; política seal→isla→reloc)
- densidad de obstáculos local (occupancy del costmap local) → factor 0.3–1.0
Implementación: suscribe salud slamko + `/cuvslam/health`, publica al
`velocity_smoother` de Nav2 (o `speed_limit` topic). También al revés: zonas nuevas del
mapa = lento (mejor mapeo), pasillos conocidos = rápido.

## 4. Depth para SEGURIDAD (crítico en robot de suelo, el TSDF no lo cubre)

- **Obstáculos negativos / cliff**: plano de suelo desde depth → celdas "void" donde el
  suelo desaparece (escaleras HACIA ABAJO = precipicio). Lethal en costmap local.
- Escaleras del mapa global = keep-out zones (el diferencial no sube).
- Obstáculos bajos + vidrio (depth los ve mejor que el slice 2D).

## 5. Ciclo de vida del mapa en robot

Auto-save al apagar; auto-localize al arrancar (BOOT→SEARCHING→LOCALIZED ya diseñado);
lugar nuevo = isla nueva sin drama; systemd/respawn + watchdog de procesos; out-of-core
sigue abierto (no bloquea escala casa/piso).

## 6. Calibración e integración física

`rs-imu-calibration` de LA unidad D455 (el ×20 nació de ahí); extrinsic cámara→base_link
medido (altura/ángulo cambian el recall — lección 40cmH/100cmH); montaje anti-vibración;
encoders accesibles; e-stop.

## 7. Lifelong activo (después)

Goal-picker que sesga rutas por zonas relocalizables cuando la incertidumbre crece
(active loop closure) + revisitas programadas para refrescar mapa.

**NO necesario para v1:** LiDAR (depth D455 cubre lo local indoor), semántica, stack alto
(congelado hasta que navegue solo).

## Orden de ejecución

1. Bag depth por la cadena nueva ✅ HECHO 2026-07-09 (veredicto honesto: cuVSLAM-Inertial
   degrada en el flash bag — 147 teleports → 35 islas honestas sin weld falso; OKVIS sigue
   siendo el provider del flash bag; diagnosticar fuga de frames punteados / 45 fps).
2. Gazebo: wheel-EKF + referee cinemático + Nav2 closed-loop + gobernador (§1+2+3 se
   validan juntos en sim, gratis).
3. Cliff/negative-obstacle desde depth (§4) — validable en el flash bag + Gazebo.
4. Robot real con kill-switch; calibración (§6) el primer día de hardware.
