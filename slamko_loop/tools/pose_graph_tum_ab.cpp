// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// pose_graph_tum_ab — a SOLVER A/B on a real EuRoC trajectory with real ATE. Builds the global
// pose-graph from a real ODOMETRY trajectory (e.g. OKVIS's drifted output) — odometry edges from
// consecutive relatives — plus LOOP-closure edges wherever the GROUND TRUTH says the path revisits
// a place (temporally distant, spatially close). The loop measurement is the GT relative (an ideal
// reloc). Then optimises with --backend ceres|gtsam and writes corrected.tum. Run it twice and
// evo_ape both vs the GT: same input graph, two solvers → isolates the backend, on real geometry.
//
//   pose_graph_tum_ab --odom okvis.tum --gt gt.tum --out corrected.tum --backend gtsam \
//       [--kf_stride 5] [--loop_dist 0.4] [--min_gap 50] [--loop_sigma_t 0.05] [--loop_sigma_r 0.02]

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slamko_loop/pose_graph.hpp"

using slamko::PoseGraph;
using slamko::PoseGraphConfig;
using slamko::PoseGraphBackend;
using slamko::SE3;
using slamko::SO3;

namespace {
struct Stamped { double t; SE3 T; };

std::vector<Stamped> readTum(const std::string& path) {
  std::vector<Stamped> v;
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    double t, x, y, z, qx, qy, qz, qw;
    if (std::sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &t, &x, &y, &z, &qx, &qy, &qz,
                    &qw) == 8) {
      Eigen::Quaterniond q(qw, qx, qy, qz);
      v.push_back({t, SE3(SO3(q.normalized()), Eigen::Vector3d(x, y, z))});
    }
  }
  return v;
}

std::string arg(int c, char** v, const std::string& k, const std::string& d) {
  for (int i = 1; i + 1 < c; ++i) if (k == v[i]) return v[i + 1];
  return d;
}

// nearest GT pose by timestamp (GT is dense; odom keyframes sample it)
const SE3& gtAt(const std::vector<Stamped>& gt, double t, std::size_t& hint) {
  while (hint + 1 < gt.size() && gt[hint + 1].t < t) ++hint;
  return gt[hint].T;
}
}  // namespace

int main(int argc, char** argv) {
  const std::string odom_p = arg(argc, argv, "--odom", "");
  const std::string gt_p = arg(argc, argv, "--gt", "");
  const std::string out_p = arg(argc, argv, "--out", "/tmp/corrected.tum");
  const std::string backend = arg(argc, argv, "--backend", "ceres");
  const int stride = std::stoi(arg(argc, argv, "--kf_stride", "5"));
  const double loop_dist = std::stod(arg(argc, argv, "--loop_dist", "0.4"));
  const int min_gap = std::stoi(arg(argc, argv, "--min_gap", "50"));
  const double lst = std::stod(arg(argc, argv, "--loop_sigma_t", "0.05"));
  const double lsr = std::stod(arg(argc, argv, "--loop_sigma_r", "0.02"));

  auto odom = readTum(odom_p);
  auto gt = readTum(gt_p);
  if (odom.size() < 10 || gt.empty()) { std::printf("FAIL: odom=%zu gt=%zu\n", odom.size(), gt.size()); return 1; }

  // keyframes = every `stride`th odom pose; pair each with its nearest-in-time GT pose.
  std::vector<Stamped> kf;
  std::vector<SE3> kf_gt;
  std::size_t hint = 0;
  for (std::size_t i = 0; i < odom.size(); i += stride) {
    kf.push_back(odom[i]);
    kf_gt.push_back(gtAt(gt, odom[i].t, hint));
  }
  const int N = static_cast<int>(kf.size());

  PoseGraphConfig cfg;
  if (backend == "gtsam") cfg.backend = PoseGraphBackend::GtsamLM;
  else if (backend == "isam2") cfg.backend = PoseGraphBackend::GtsamISAM2;
  const bool incremental = std::string(arg(argc, argv, "--incremental", "0")) == "1";

  // pre-compute loop edges (temporally distant, spatially close per GT) so both modes use the same.
  std::vector<std::array<int, 2>> loop_pairs;
  for (int i = 0; i < N; ++i)
    for (int j = i + min_gap; j < N; ++j)
      if ((kf_gt[i].translation() - kf_gt[j].translation()).norm() < loop_dist) {
        loop_pairs.push_back({i, j});
        j += min_gap;
      }
  std::printf("backend=%s  keyframes=%d  loop_edges=%zu  mode=%s\n", backend.c_str(), N,
              loop_pairs.size(), incremental ? "incremental" : "batch");

  PoseGraph pg(cfg);
  PoseGraph::Result res;
  if (!incremental) {
    for (int i = 0; i < N; ++i) pg.addKeyframe(i, kf[i].T);
    pg.setAnchor(0);
    for (int i = 0; i + 1 < N; ++i)
      pg.addOdometryEdge(i, i + 1, kf[i].T.inverse() * kf[i + 1].T, 0.02, 0.01);
    for (const auto& lp : loop_pairs)
      pg.addLoopEdge(lp[0], lp[1], kf_gt[lp[0]].inverse() * kf_gt[lp[1]], lst, lsr);
    res = pg.optimize();
  } else {
    // INCREMENTAL: add one keyframe (+ its odom edge + any loop that closes now) per step, optimise
    // each step, TIME each optimise() — this is where iSAM2 (O(touched)) beats Ceres (O(graph)).
    std::vector<std::vector<int>> loops_at(N);  // loop indices that close AT keyframe j
    for (std::size_t k = 0; k < loop_pairs.size(); ++k) loops_at[loop_pairs[k][1]].push_back((int)k);
    double t_total = 0.0, t_last10 = 0.0, t_first10 = 0.0;
    pg.addKeyframe(0, kf[0].T);
    pg.setAnchor(0);
    for (int i = 0; i < N; ++i) {
      if (i > 0) {
        pg.addKeyframe(i, kf[i].T);
        pg.addOdometryEdge(i - 1, i, kf[i - 1].T.inverse() * kf[i].T, 0.02, 0.01);
      }
      for (int k : loops_at[i])
        pg.addLoopEdge(loop_pairs[k][0], loop_pairs[k][1],
                       kf_gt[loop_pairs[k][0]].inverse() * kf_gt[loop_pairs[k][1]], lst, lsr);
      const auto c0 = std::chrono::steady_clock::now();
      res = pg.optimize();
      const double ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - c0).count();
      t_total += ms;
      if (i < 10) t_first10 += ms;
      if (i >= N - 10) t_last10 += ms;
    }
    std::printf("incremental timing: total %.1f ms, mean %.2f ms/step; first-10 mean %.2f ms, "
                "last-10 mean %.2f ms (graph %dx bigger at the end)\n",
                t_total, t_total / N, t_first10 / 10.0, t_last10 / 10.0, N);
  }
  std::printf("optimize: %d nodes, %d odom, %d loops, cost %.3g -> %.3g\n", res.num_nodes,
              res.num_odom, res.num_loops, res.initial_cost, res.final_cost);

  std::ofstream of(out_p);
  for (int i = 0; i < N; ++i) {
    const SE3 T = pg.pose(i);
    const Eigen::Quaterniond q = T.so3().unit_quaternion();
    const Eigen::Vector3d t = T.translation();
    of << std::fixed << kf[i].t << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x()
       << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  std::printf("wrote %s (%d poses)\n", out_p.c_str(), N);
  return 0;
}
