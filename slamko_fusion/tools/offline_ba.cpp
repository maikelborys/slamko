// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// offline_ba — apply GtsamGlobalSmoother to a saved Atlas. Per-submap BA refines
// each submap's local KF poses + landmarks using its kf_obs reprojection factors;
// the anchor stays fixed (the supervisor's pose-graph already set it on closure).
// The refined Atlas is saved alongside the input so corrected-trajectory ATE can be
// compared (per docs/PLAN_BA_GLOBAL.md Phase D). Pure offline — no ROS, no live
// supervisor surgery — so the BA can be validated end-to-end on real data without
// touching the never-lost wiring yet.
//
// Usage:
//   offline_ba <map_in_dir> <map_out_dir> [--no-imu]
//
// The input dir must contain submap_*.smap (SMP3 with kf_obs) + calib.txt.
// --no-imu forces visual-only BA even when SMP5 IMU windows are present (A/B baseline).
// calib.txt format (one line, ASCII):
//   fx fy cx cy baseline  tx ty tz qx qy qz qw
//   (the last 7 fields are T_BS — cam->body — quaternion + translation)

#include <cstdio>
#include <fstream>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slamko_core/se3.hpp"
#include "slamko_core/stereo_observation.hpp"
#include "slamko_core/submap.hpp"
#include "slamko_core/submap_io.hpp"
#include "slamko_fusion/gtsam_global_smoother.hpp"

namespace {
bool loadCalib(const std::string& path, slamko::StereoCalib& K, slamko::SE3& T_BS) {
  std::ifstream f(path);
  if (!f) return false;
  double tx, ty, tz, qx, qy, qz, qw;
  f >> K.fx >> K.fy >> K.cx >> K.cy >> K.baseline
    >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
  if (!f) return false;
  const Eigen::Quaterniond q(qw, qx, qy, qz);
  T_BS = slamko::SE3(slamko::SO3(q), Eigen::Vector3d(tx, ty, tz));
  return true;
}

// Build a per-submap BA input in LOCAL frame: anchor stays out, gauge = first KF.
// Each submap is independent — this refines intra-submap geometry only (the pose-
// graph weld already handled the cross-submap anchor correction at closure time).
// When SMP5 per-KF IMU windows are present, attach CombinedImuFactor constraints
// between consecutive KFs (Phase B.2). Visual-only BA degrades ATE on real data
// (PLAN_BA_GLOBAL.md D.1 — V1_03 37→82 cm); IMU factors lock metric scale + gravity
// rotation + bias drift, which is what makes the BA actually move ATE in the right
// direction. Velocities are bootstrapped from consecutive KF translations (the local
// smoother's truth post-optimize isn't persisted yet; finite-diff is good enough as
// LM iterates and refines the velocity variable anyway). Biases initialize to zero
// for the same reason — the bias random-walk priors keep them small.
slamko::GlobalBAInput buildPerSubmapInput(const slamko::SubMap& sm,
                                          const slamko::StereoCalib& K,
                                          const slamko::SE3& T_BS,
                                          bool use_imu) {
  slamko::GlobalBAInput in;
  in.calib = K;
  in.T_BS  = T_BS;
  in.pixel_sigma = 1.0;
  in.max_iters   = 30;
  for (const auto& kf : sm.keyframes) in.keyframes.emplace_back(kf.id, kf.T_WB);
  for (const auto& l  : sm.landmarks) in.landmarks.emplace_back(l.id, l.position);
  // Observations from kf_obs (aligned 1:1 with keyframes).
  for (std::size_t k = 0; k < sm.keyframes.size() && k < sm.kf_obs.size(); ++k) {
    const auto& ko = sm.kf_obs[k];
    const bool stereo = ko.hasStereo();
    for (int i = 0; i < ko.size(); ++i) {
      slamko::GlobalBAObservation o;
      o.kf_id        = sm.keyframes[k].id;
      o.landmark_id  = ko.landmark_ids[i];
      o.uv_left      = ko.uv.row(i).transpose();
      // Per-row stereo check: mixed-stereo KFs use NaN-x for mono rows (matches
      // StereoObservation's convention). v1 smoother skips mono rows anyway.
      if (stereo && std::isfinite(ko.uv_right(i, 0)))
        o.uv_right = ko.uv_right.row(i).transpose();
      in.observations.push_back(o);
    }
  }
  if (!sm.keyframes.empty()) in.anchor_kf = sm.keyframes.front().id;

  // IMU windows (SMP5). Only when at least one KF carries `imu_since_prev` — older
  // Atlases (SMP3/SMP4) fall through to visual-only. `--no-imu` also forces this off
  // so we have a clean A/B baseline on the same map.
  bool any_imu = false;
  for (const auto& ko : sm.kf_obs) if (ko.hasImu()) { any_imu = true; break; }
  if (use_imu && any_imu && sm.keyframes.size() >= 2) {
    in.imu_params = slamko::ImuParams{};  // EuRoC-tuned defaults from imu_sample.hpp
    // Initial velocity per KF from consecutive-KF translation / dt. The first and
    // last KFs share the closest neighbor estimate to keep all KFs that are window
    // endpoints covered (CombinedImuFactor binds (X,V,B) at BOTH endpoints).
    for (std::size_t k = 0; k < sm.keyframes.size(); ++k) {
      Eigen::Vector3d v(0, 0, 0);
      // Pick the consecutive pair that exists. For interior KFs the centered diff is
      // more accurate but the IMU factor will refine the estimate anyway.
      const std::size_t kp = (k + 1 < sm.keyframes.size()) ? k + 1 : k;
      const std::size_t km = (k == 0) ? 0 : k - 1;
      if (kp != km) {
        const double dt =
            sm.keyframes[kp].timestamp - sm.keyframes[km].timestamp;
        if (dt > 1e-6)
          v = (sm.keyframes[kp].T_WB.translation() -
               sm.keyframes[km].T_WB.translation()) / dt;
      }
      in.velocities.emplace_back(sm.keyframes[k].id, v);
      in.biases.emplace_back(sm.keyframes[k].id, slamko::ImuBias{});  // zero init
    }
    // IMU windows: each KF's `imu_since_prev` spans (KF k-1) → (KF k). First KF stores [].
    for (std::size_t k = 1; k < sm.keyframes.size() && k < sm.kf_obs.size(); ++k) {
      if (!sm.kf_obs[k].hasImu()) continue;
      slamko::GlobalBAImuWindow w;
      w.kf_from = sm.keyframes[k - 1].id;
      w.kf_to   = sm.keyframes[k].id;
      w.samples = sm.kf_obs[k].imu_since_prev;
      in.imu_windows.push_back(w);
    }
  }
  return in;
}

void writeBack(const slamko::GlobalBAOutput& out, slamko::SubMap& sm) {
  std::unordered_map<std::uint64_t, slamko::SE3> kf_refined;
  for (const auto& kv : out.keyframes) kf_refined[kv.first] = kv.second;
  std::unordered_map<std::uint64_t, Eigen::Vector3d> lm_refined;
  for (const auto& kv : out.landmarks) lm_refined[kv.first] = kv.second;
  for (auto& kf : sm.keyframes) {
    auto it = kf_refined.find(kf.id);
    if (it != kf_refined.end()) kf.T_WB = it->second;
  }
  for (auto& lm : sm.landmarks) {
    auto it = lm_refined.find(lm.id);
    if (it != lm_refined.end()) lm.position = it->second;
  }
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr,
        "usage: offline_ba <map_in_dir> <map_out_dir> [--no-imu]\n");
    return 1;
  }
  const std::string map_in  = argv[1];
  const std::string map_out = argv[2];
  bool use_imu = true;
  for (int i = 3; i < argc; ++i)
    if (std::string(argv[i]) == "--no-imu") use_imu = false;
  std::printf("offline_ba: use_imu=%s\n", use_imu ? "true" : "false");

  slamko::StereoCalib K{};
  slamko::SE3 T_BS;
  if (!loadCalib(map_in + "/calib.txt", K, T_BS)) {
    std::fprintf(stderr, "cannot read %s/calib.txt — the offline BA needs it to build "
                         "reprojection factors\n", map_in.c_str());
    return 2;
  }
  std::printf("calib  fx=%.2f fy=%.2f cx=%.2f cy=%.2f baseline=%.4f m\n",
              K.fx, K.fy, K.cx, K.cy, K.baseline);

  std::vector<slamko::SubMap> maps;
  if (!slamko::loadSubMaps(maps, map_in)) {
    std::fprintf(stderr, "cannot load Atlas from %s\n", map_in.c_str());
    return 3;
  }
  std::printf("loaded %zu submaps from %s\n", maps.size(), map_in.c_str());

  slamko_fusion::GtsamGlobalSmoother smoother;

  int n_refined = 0, n_skipped = 0;
  double tot_init = 0.0, tot_final = 0.0;
  for (auto& sm : maps) {
    if (sm.keyframes.empty() || sm.kf_obs.empty() ||
        sm.kf_obs.size() != sm.keyframes.size()) {
      std::printf("  submap %lu: SKIP (no observations stored — pre-SMP3 or legacy)\n",
                  (unsigned long)sm.id);
      ++n_skipped;
      continue;
    }
    const auto in  = buildPerSubmapInput(sm, K, T_BS, use_imu);
    if (in.observations.empty()) {
      std::printf("  submap %lu: SKIP (no stereo observations)\n", (unsigned long)sm.id);
      ++n_skipped;
      continue;
    }
    const auto out = smoother.optimize(in);
    std::printf("  submap %lu: KFs=%zu lms=%zu obs=%zu imu_windows=%zu  initial=%.2f "
                "→ final=%.2f (iters=%d %s)\n",
                (unsigned long)sm.id, sm.keyframes.size(), sm.landmarks.size(),
                in.observations.size(), in.imu_windows.size(),
                out.initial_cost, out.final_cost,
                out.iterations, out.converged ? "OK" : "no-improve");
    if (out.converged) {
      writeBack(out, sm);
      ++n_refined;
      tot_init += out.initial_cost;
      tot_final += out.final_cost;
    } else {
      ++n_skipped;
    }
  }

  if (!slamko::saveSubMaps(maps, map_out)) {
    std::fprintf(stderr, "FAILED to save refined Atlas to %s\n", map_out.c_str());
    return 4;
  }
  // Copy the calib forward so the refined Atlas is self-contained.
  std::ifstream src(map_in + "/calib.txt", std::ios::binary);
  std::ofstream dst(map_out + "/calib.txt", std::ios::binary);
  dst << src.rdbuf();

  std::printf("DONE  refined=%d skipped=%d  total reprojection cost %.2f → %.2f\n",
              n_refined, n_skipped, tot_init, tot_final);
  return 0;
}
