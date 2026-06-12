// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// provider_chain_offline — the P-A reproducible gate (no ROS, no provider
// bring-up): feed a recorded provider trajectory (TUM: t x y z qx qy qz qw,
// e.g. ~/coding/klt_vo/results/d455/okvis_{Suave,Escaleras}.tum) through the
// EXACT same path the live node uses — ProviderChain decimation -> relative
// edges -> PoseGraph -> optimize() — then reconstruct the fused keyframe poses
// and compare against the input poses at the same timestamps.
//
// With no global constraints the chain is the exact global minimum, so the
// fused trajectory must reproduce the provider to numerical precision: any
// divergence is a bug in the edge math / information weighting / solver
// round-trip. Exit 1 when max position error > --tol (default 1e-3 m).
//
//   provider_chain_offline --tum okvis_Suave.tum [--out fused.tum]
//       [--min-trans 0.10] [--min-rot 0.10] [--max-dt 1.0] [--tol 1e-3]

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slamko_core/odometry_provider.hpp"
#include "slamko_core/se3.hpp"
#include "slamko_loop/pose_graph.hpp"

int main(int argc, char** argv) {
  std::string tum_path, out_path;
  slamko::ProviderChainConfig ccfg;
  double tol = 1e-3;
  for (int i = 1; i < argc - 1; ++i) {
    if (!std::strcmp(argv[i], "--tum")) tum_path = argv[++i];
    else if (!std::strcmp(argv[i], "--out")) out_path = argv[++i];
    else if (!std::strcmp(argv[i], "--min-trans")) ccfg.kf_min_translation = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--min-rot")) ccfg.kf_min_rotation = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--max-dt")) ccfg.kf_max_dt = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--tol")) tol = std::atof(argv[++i]);
  }
  if (tum_path.empty()) {
    std::fprintf(stderr, "usage: provider_chain_offline --tum <traj.tum> [--out f.tum] "
                         "[--min-trans m] [--min-rot rad] [--max-dt s] [--tol m]\n");
    return 2;
  }

  std::ifstream in(tum_path);
  if (!in) { std::fprintf(stderr, "cannot open %s\n", tum_path.c_str()); return 2; }

  slamko::ProviderChain chain(ccfg);
  slamko::PoseGraph pg;
  // Provider keyframe poses (raw, provider frame) keyed by chain id — the
  // comparison reference.
  std::map<std::uint64_t, std::pair<double, slamko::SE3>> ref;

  std::size_t n_samples = 0;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    double t, x, y, z, qx, qy, qz, qw;
    if (!(ss >> t >> x >> y >> z >> qx >> qy >> qz >> qw)) continue;
    slamko::ProviderSample s;
    s.t = t;
    s.T_OB = slamko::SE3(slamko::SO3(Eigen::Quaterniond(qw, qx, qy, qz).normalized()),
                         Eigen::Vector3d(x, y, z));
    ++n_samples;

    const bool first = !chain.hasKeyframe();
    const auto edge = chain.feed(s);
    if (first) {
      pg.addKeyframe(0, s.T_OB);
      pg.setAnchor(0);
      ref[0] = {s.t, s.T_OB};
    } else if (edge) {
      pg.addKeyframe(edge->to, pg.pose(edge->from) * edge->T_from_to);
      pg.addEdge(edge->from, edge->to, edge->T_from_to, edge->information, false);
      ref[edge->to] = {s.t, s.T_OB};
    }
  }
  if (ref.size() < 2) { std::fprintf(stderr, "too few keyframes\n"); return 2; }

  auto compare = [&](const char* tag) {
    double max_t = 0.0, sum_sq = 0.0, max_r = 0.0;
    for (const auto& [id, tr] : ref) {
      const slamko::SE3 d = tr.second.inverse() * pg.pose(id);
      const double et = d.translation().norm();
      const double er = d.so3().log().norm();
      max_t = std::max(max_t, et);
      max_r = std::max(max_r, er);
      sum_sq += et * et;
    }
    std::printf("%-14s max_pos=%.3e m  rmse_pos=%.3e m  max_rot=%.3e rad\n", tag,
                max_t, std::sqrt(sum_sq / double(ref.size())), max_r);
    return max_t;
  };

  std::printf("samples=%zu keyframes=%zu edges=%zu\n", n_samples, pg.numNodes(),
              pg.numEdges());
  compare("pre-optimize");
  const auto res = pg.optimize();
  std::printf("optimize: converged=%d iters=%d cost %.3e -> %.3e\n",
              int(res.converged), res.iterations, res.initial_cost, res.final_cost);
  const double max_err = compare("post-optimize");

  if (!out_path.empty()) {
    std::FILE* f = std::fopen(out_path.c_str(), "w");
    if (f) {
      for (const auto& [id, tr] : ref) {
        const slamko::SE3 T = pg.pose(id);
        const Eigen::Quaterniond q = T.so3().unit_quaternion();
        const Eigen::Vector3d p = T.translation();
        std::fprintf(f, "%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", tr.first,
                     p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
      }
      std::fclose(f);
    }
  }

  const bool pass = max_err <= tol;
  std::printf("%s (max %.3e m vs tol %.3e m)\n", pass ? "PASS" : "FAIL", max_err, tol);
  return pass ? 0 : 1;
}
