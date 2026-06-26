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
  PoseGraph pg(cfg);
  for (int i = 0; i < N; ++i) pg.addKeyframe(i, kf[i].T);          // init = drifted odom
  pg.setAnchor(0);
  for (int i = 0; i + 1 < N; ++i)                                   // odometry edges (real relatives)
    pg.addOdometryEdge(i, i + 1, kf[i].T.inverse() * kf[i + 1].T, 0.02, 0.01);

  // loop edges from GT revisits: a temporally-distant, spatially-close pair gets an ideal closure.
  int loops = 0;
  for (int i = 0; i < N; ++i)
    for (int j = i + min_gap; j < N; ++j)
      if ((kf_gt[i].translation() - kf_gt[j].translation()).norm() < loop_dist) {
        pg.addLoopEdge(i, j, kf_gt[i].inverse() * kf_gt[j], lst, lsr);  // GT-relative = ideal reloc
        ++loops;
        j += min_gap;  // thin out dense overlaps
      }
  std::printf("backend=%s  keyframes=%d  loop_edges=%d\n", backend.c_str(), N, loops);

  const auto res = pg.optimize();
  std::printf("optimize: %d nodes, %d odom, %d loops, cost %.3g -> %.3g, %d iters\n",
              res.num_nodes, res.num_odom, res.num_loops, res.initial_cost, res.final_cost,
              res.iterations);

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
