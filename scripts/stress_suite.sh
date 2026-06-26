#!/usr/bin/env bash
# Stress suite for the cuVSLAM→slamko stack. Runs a SERIES of failure-mode scenarios
# serially (reaping cleanly between — zombie discipline), then prints an HONEST behavior
# table: did it CRASH, how the MAP behaved (components/submaps), whether the never-lost
# quality-break/Atlas fired + recovered, and how badly the PROVIDER (cuVSLAM) diverged
# (independent of slamko). The point is robustness, not ATE.
# cuVSLAM runs only on dim-consistent 848 bags; the flashing bag goes through the splitter.
set +e
cd ~/coding/slamko
RES=/tmp/stress_results
rm -rf "$RES"; mkdir -p "$RES"

reap() { pkill -KILL -f 'visual_slam|cuvslam_container|component_container|robot_state_publisher|provider_fusion_node|cuvslam_body_tf|d455_splitter_auto|cam_info_inject|ros2 bag play' 2>/dev/null; sleep 3; }

# Parse one run's outputs → one metrics line in $RES/table.txt
metrics() {
  local name="$1" out="$2"
  local crash=$(grep -ciE 'what\(\)|terminate called|segmentation|core dumped|loading shared' "$out/fusion.log" 2>/dev/null)
  local comps=$(tail -n +2 "$out/map/components.csv" 2>/dev/null | cut -d, -f2 | sort -u | wc -l)
  local subs=$(ls "$out"/map/submap_*.smap 2>/dev/null | wc -l)
  local qlost=$(grep -cE 'QUALITY LOST' "$out/fusion.log" 2>/dev/null)
  local qrec=$(grep -cE 'QUALITY RECOVERED' "$out/fusion.log" 2>/dev/null)
  local breaks=$(grep -cE 'ATLAS BREAK' "$out/fusion.log" 2>/dev/null)
  local loops=$(grep -cE '> verified' "$out/fusion.log" 2>/dev/null)
  local r0=$(grep -cE 'R0 gate' "$out/fusion.log" 2>/dev/null)
  local div=$(python3 -c "
import numpy as np
try:
  d=np.loadtxt('$out/provider.tum'); p=d[:,1:4]
  print('%.1f'%np.linalg.norm(p-p[0],axis=1).max())
except: print('n/a')" 2>/dev/null)
  printf "%-22s | crash:%s | prov|disp|max:%sm | comp:%s subs:%s | qLOST:%s qREC:%s break:%s | loops:%s R0:%s\n" \
    "$name" "$([ "$crash" = 0 ] && echo NO || echo YES!)" "$div" "$comps" "$subs" "$qlost" "$qrec" "$breaks" "$loops" "$r0" \
    | tee -a "$RES/table.txt"
}

echo "######## STRESS SUITE — cuVSLAM→slamko ########" | tee "$RES/table.txt"

# --- S1: GENTLE baseline (40cmH) — expect clean: 1 comp, loop closes, few/no breaks ---
echo "[S1] gentle baseline (40cmH)..." ; reap
SC=false QSOFT=true bash scripts/run_slamko_cuvslam_casa.sh /mnt/data/bno_ab/CASA1_40cmH_Stereo60_RGB30_BNO_848_trim 0.5 "$RES/s1" >/dev/null 2>&1
metrics "S1 gentle 40cm" "$RES/s1"

# --- S2: AGGRESSIVE return (100cmH) — different-heading return + faster motion ---
echo "[S2] aggressive return (100cmH)..." ; reap
SC=true QSOFT=true bash scripts/run_slamko_cuvslam_casa.sh /mnt/data/bno_ab/CASA1_100cmH_Stereo60_RGB30_BNO_848_trim 0.5 "$RES/s2" >/dev/null 2>&1
metrics "S2 aggressive 100cm" "$RES/s2"

# --- S3: CATASTROPHIC provider divergence + VOLUMETRIC (flashbag via splitter) ---
echo "[S3] catastrophic + volumetric (flashbag)..." ; reap
QSOFT=true bash scripts/run_slamko_cuvslam_volumetric.sh /mnt/data/d455_bags/casa_084815_flashbno_trim 0.5 "$RES/s3" >/dev/null 2>&1
metrics "S3 catastrophic+vol" "$RES/s3"

reap
echo "######## DONE ########" | tee -a "$RES/table.txt"
echo "=== HONEST BEHAVIOR TABLE ==="; cat "$RES/table.txt"
