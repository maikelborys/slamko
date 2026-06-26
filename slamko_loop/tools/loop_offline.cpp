// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// loop_offline — the OFFLINE loop-closure + optimization driver (Milestone 2). Runs the
// whole "loop closure + optimization, separate" layer on a recorded run's sealed submaps:
//
//   loadSubMaps(dir)                          (the lifelong substrate the VIO dumped)
//     → build the global pose-graph: nodes = KF world poses (anchor·local);
//       odometry edges = the relative VIO chain (intra- + inter-submap)
//     → detect loops: XFeatRelocalizer over an INCREMENTAL DB that only holds submaps
//       ≥ min_gap older than the query (so every match is a genuine revisit, never a
//       self/neighbor); EigenPlaces VPR retrieval + XFeat-PnP geometric verify
//     → weld: edge query→matchedKF, T_from_to = T_query_match⁻¹ · matchedKF.T_WB
//       (anchor-invariant — proven in test_pose_graph WeldMathRecoversRelative)
//     → PoseGraph::optimize()  (redistribute drift, close the loop)
//     → write baseline.tum + corrected.tum + metrics (end-to-start gap before/after).
//
//   loop_offline --smap_dir <in> --out <prefix> --fx F --fy F --cx C --cy C
//                [--min_gap 3] [--min_inliers 25] [--vpr_top_n 10] [--loop_sigma_t 0.1]
//
// Depends on slamko_core + slamko_loop only (Hard Rule #2).

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "slamko_core/submap_io.hpp"
#include "slamko_core/features.hpp"
#include "slamko_loop/pose_graph.hpp"
#include "slamko_loop/xfeat_relocalizer.hpp"

using namespace slamko;

namespace {

std::uint64_t gid(std::size_t s, std::size_t k) {
  return (static_cast<std::uint64_t>(s) << 32) | static_cast<std::uint32_t>(k);
}

// Build a relocalization query Features from keyframe k of a submap: per observed
// landmark, its pixel uv + the 64-D XFeat descriptor (via the submap's id→row index)
// + the per-KF EigenPlaces VPR vector. Mirrors xfeat_relocalizer's train build.
Features buildQuery(const SubMap& sm, int k,
                    const std::unordered_map<std::uint64_t, int>& lid2row) {
  const auto& ko = sm.kf_obs[k];
  const int N = ko.size();
  const int D = sm.descriptors.cols() > 0 ? static_cast<int>(sm.descriptors.cols()) : 64;
  Features q;
  q.keypoints.resize(N, 3);
  q.descriptors.resize(N, D);
  int row = 0;
  for (int i = 0; i < N; ++i) {
    auto it = lid2row.find(ko.landmark_ids[i]);
    if (it == lid2row.end() || it->second < 0) continue;
    q.keypoints(row, 0) = ko.uv(i, 0);
    q.keypoints(row, 1) = ko.uv(i, 1);
    q.keypoints(row, 2) = 1.0f;
    q.descriptors.row(row) = sm.descriptors.row(it->second);
    ++row;
  }
  q.keypoints.conservativeResize(row, 3);
  q.descriptors.conservativeResize(row, D);
  q.global_descriptor = ko.global_descriptor;
  return q;
}

double cosine(const Eigen::VectorXf& a, const Eigen::VectorXf& b) {
  if (a.size() == 0 || a.size() != b.size()) return -1.0;
  const double na = a.norm(), nb = b.norm();
  if (na < 1e-9 || nb < 1e-9) return -1.0;
  return static_cast<double>(a.dot(b)) / (na * nb);
}

std::string argval(int argc, char** argv, const std::string& key, const std::string& def) {
  for (int i = 1; i + 1 < argc; ++i) if (key == argv[i]) return argv[i + 1];
  return def;
}

void writeTum(const std::string& path, const std::vector<std::pair<std::uint64_t, SE3>>& poses) {
  std::ofstream o(path);
  double t = 0.0;
  for (const auto& [id, T] : poses) {
    const auto& p = T.translation();
    const Eigen::Quaterniond q = T.so3().unit_quaternion();
    o << std::fixed << t << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' '
      << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
    t += 1.0;
    (void)id;
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::string smap_dir = argval(argc, argv, "--smap_dir", "");
  const std::string out      = argval(argc, argv, "--out", "/tmp/loop");
  if (smap_dir.empty()) { std::printf("usage: loop_offline --smap_dir <dir> --out <prefix> --fx..\n"); return 2; }

  XFeatRelocConfig rc;
  rc.fx = std::stod(argval(argc, argv, "--fx", "165.41"));
  rc.fy = std::stod(argval(argc, argv, "--fy", "165.41"));
  rc.cx = std::stod(argval(argc, argv, "--cx", "376.0"));
  rc.cy = std::stod(argval(argc, argv, "--cy", "240.0"));
  rc.use_vpr      = true;
  rc.vpr_top_n    = std::stoi(argval(argc, argv, "--vpr_top_n", "10"));
  rc.match_ratio  = 0.9f;
  rc.mutual_check = true;
  rc.min_inlier_ratio = 0.30;
  rc.min_inliers  = std::stoi(argval(argc, argv, "--min_inliers", "25"));
  const int min_gap = std::stoi(argval(argc, argv, "--min_gap", "3"));
  // Loop detection is a SUBMAP-level operation; querying every KF is wasteful (and
  // intractable when the VIO over-keyframes — magistrale1 made 14429 KFs). Query only
  // every Nth KF (~a few per submap). Decouples loop-detection cost from odometry KF
  // density without touching the trajectory (all KFs stay as pose-graph nodes).
  const int query_stride = std::stoi(argval(argc, argv, "--query_stride", "15"));
  const double loop_st = std::stod(argval(argc, argv, "--loop_sigma_t", "0.10"));
  const double loop_sr = std::stod(argval(argc, argv, "--loop_sigma_r", "0.05"));

  std::vector<SubMap> maps;
  if (!loadSubMaps(maps, smap_dir)) { std::printf("FAIL: loadSubMaps('%s')\n", smap_dir.c_str()); return 1; }
  std::printf("loaded %zu submaps\n", maps.size());

  // per-submap landmark-id → descriptor-row index
  std::vector<std::unordered_map<std::uint64_t, int>> lid2row(maps.size());
  for (std::size_t s = 0; s < maps.size(); ++s)
    for (const auto& lm : maps[s].landmarks)
      if (lm.descriptor_row >= 0) lid2row[s][lm.id] = lm.descriptor_row;

  // ---- build the global pose-graph (odometry chain) ----
  // --backend ceres|gtsam selects the solver (gtsam needs -DSLAMKO_LOOP_WITH_GTSAM=ON; else it
  // falls back to Ceres + warns). Same submaps + same edges -> isolates the SOLVER for an ATE A/B.
  slamko::PoseGraphConfig pgcfg;
  const std::string backend = argval(argc, argv, "--backend", "ceres");
  if (backend == "gtsam") pgcfg.backend = slamko::PoseGraphBackend::GtsamLM;
  std::printf("pose-graph backend: %s\n", backend.c_str());
  PoseGraph pg(pgcfg);
  for (std::size_t s = 0; s < maps.size(); ++s)
    for (std::size_t k = 0; k < maps[s].keyframes.size(); ++k)
      pg.addKeyframe(gid(s, k), maps[s].anchor * maps[s].keyframes[k].T_WB);
  pg.setAnchor(gid(0, 0));

  int n_odom = 0;
  for (std::size_t s = 0; s < maps.size(); ++s) {
    const auto& kf = maps[s].keyframes;
    for (std::size_t k = 0; k + 1 < kf.size(); ++k) {  // intra-submap (local; anchor cancels)
      pg.addOdometryEdge(gid(s, k), gid(s, k + 1), kf[k].T_WB.inverse() * kf[k + 1].T_WB, 0.02, 0.01);
      ++n_odom;
    }
    if (s + 1 < maps.size() && !kf.empty() && !maps[s + 1].keyframes.empty()) {  // inter-submap (world)
      const SE3 A = maps[s].anchor * kf.back().T_WB;
      const SE3 B = maps[s + 1].anchor * maps[s + 1].keyframes.front().T_WB;
      pg.addOdometryEdge(gid(s, kf.size() - 1), gid(s + 1, 0), A.inverse() * B, 0.02, 0.01);
      ++n_odom;
    }
  }

  // baseline (open-loop) end-to-start gap, un-aligned (Hard Rule #5).
  auto endStartGap = [&]() {
    const SE3 first = pg.pose(gid(0, 0));
    const std::size_t S = maps.size() - 1;
    const SE3 last = pg.pose(gid(S, maps[S].keyframes.size() - 1));
    return (last.translation() - first.translation()).norm();
  };
  const double gap_before = endStartGap();
  writeTum(out + "_baseline.tum", pg.poses());

  // ---- loop detection: incremental causal DB (only submaps ≥ min_gap older) ----
  XFeatRelocalizer reloc(rc);
  std::size_t db_next = 0;
  int n_loops = 0, n_queries = 0, n_found = 0;
  std::size_t kf_seen = 0;  // global KF counter for query subsampling
  for (std::size_t s_q = 0; s_q < maps.size(); ++s_q) {
    while (db_next + static_cast<std::size_t>(min_gap) <= s_q) reloc.addSubMap(maps[db_next++]);
    if (db_next == 0) { kf_seen += maps[s_q].kf_obs.size(); continue; }
    for (std::size_t k = 0; k < maps[s_q].kf_obs.size(); ++k) {
      if ((kf_seen++ % static_cast<std::size_t>(query_stride)) != 0) continue;  // subsample queries
      Features q = buildQuery(maps[s_q], static_cast<int>(k), lid2row[s_q]);
      if (!q.hasGlobalDescriptor() || q.size() < 8) continue;
      ++n_queries;
      const RelocResult r = reloc.relocalize(q);
      if (!r.found || r.num_inliers < rc.min_inliers) continue;
      ++n_found;
      // resolve matched submap index from its id
      std::size_t s_db = maps.size();
      for (std::size_t i = 0; i < db_next; ++i) if (maps[i].id == r.submap_id) { s_db = i; break; }
      if (s_db == maps.size()) continue;
      // matched KF = max VPR-cosine in the matched submap
      int kbest = 0; double best = -2.0;
      for (std::size_t j = 0; j < maps[s_db].kf_obs.size(); ++j) {
        const double c = cosine(q.global_descriptor, maps[s_db].kf_obs[j].global_descriptor);
        if (c > best) { best = c; kbest = static_cast<int>(j); }
      }
      const SE3 T_from_to = r.T_query_match.inverse() * maps[s_db].keyframes[kbest].T_WB;
      pg.addLoopEdge(gid(s_q, k), gid(s_db, kbest), T_from_to, loop_st, loop_sr);
      ++n_loops;
      std::printf("  LOOP: submap %zu kf %zu  ->  submap %zu kf %d  (inliers %d, vpr %.3f)\n",
                  s_q, k, s_db, kbest, r.num_inliers, best);
    }
  }

  // ---- optimize + report ----
  const PoseGraph::Result res = pg.optimize();
  const double gap_after = endStartGap();
  writeTum(out + "_corrected.tum", pg.poses());

  std::printf("\n=== loop_offline metrics ===\n");
  std::printf("submaps=%zu  KF-nodes=%zu  odom_edges=%d  loop_edges=%d\n",
              maps.size(), pg.numNodes(), n_odom, n_loops);
  std::printf("queries=%d  reloc_found=%d  loops_accepted=%d\n", n_queries, n_found, n_loops);
  std::printf("pose_graph: converged=%d  cost %.3g -> %.3g  iters=%d\n",
              res.converged, res.initial_cost, res.final_cost, res.iterations);
  std::printf("END-TO-START GAP: %.2f m (before) -> %.2f m (after)   [%.0f%% closed]\n",
              gap_before, gap_after, gap_before > 1e-6 ? 100.0 * (1.0 - gap_after / gap_before) : 0.0);

  std::ofstream mj(out + "_metrics.json");
  mj << "{\n  \"submaps\": " << maps.size() << ",\n  \"kf_nodes\": " << pg.numNodes()
     << ",\n  \"odom_edges\": " << n_odom << ",\n  \"loop_edges\": " << n_loops
     << ",\n  \"queries\": " << n_queries << ",\n  \"reloc_found\": " << n_found
     << ",\n  \"converged\": " << (res.converged ? "true" : "false")
     << ",\n  \"cost_initial\": " << res.initial_cost << ",\n  \"cost_final\": " << res.final_cost
     << ",\n  \"end_to_start_gap_before_m\": " << gap_before
     << ",\n  \"end_to_start_gap_after_m\": " << gap_after << "\n}\n";
  return 0;
}
