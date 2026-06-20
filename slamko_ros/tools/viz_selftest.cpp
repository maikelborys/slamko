// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// viz_selftest — a no-ROS runtime exercise of VizSink (only built with
// SLAMKO_WITH_RERUN). Feeds synthetic keyframes / submaps / typed edges through
// every VizSink method and records to the .rrd path in argv[1] (default
// /tmp/slamko_viz_selftest.rrd). Open it with `rerun <file>.rrd` to scrub the
// fake run — both the over-video window and the 3D map window. This is the smoke
// test that proves the Rerun logging path actually emits data end-to-end.
//   build: colcon build --packages-select slamko_ros --cmake-args -DSLAMKO_WITH_RERUN=ON
//   run  : ros2 run slamko_ros viz_selftest /tmp/x.rrd   (or run the binary directly)

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "slamko_core/se3.hpp"
#include "slamko_ros/viz_sink.hpp"

int main(int argc, char** argv) {
  const std::string out = argc > 1 ? argv[1] : "/tmp/slamko_viz_selftest.rrd";
  slamko::VizSink viz;
  if (!viz.init("slamko_selftest", out)) {
    std::fprintf(stderr, "VizSink.init('%s') failed (no SLAMKO_WITH_RERUN?)\n", out.c_str());
    return 1;
  }
  viz.setSessionTransform(slamko::SE3(), false);

  // A synthetic "prior" backdrop: a grid of points + a couple of submap clouds.
  std::vector<Eigen::Vector3d> prior;
  for (int i = -20; i <= 20; ++i)
    for (int j = -20; j <= 20; ++j)
      prior.push_back({i * 0.1, j * 0.1, 0.0});
  viz.logPriorCloud(0, prior);

  std::vector<std::array<Eigen::Vector3d, 2>> chain, soft, loop, prioredge, cand;
  Eigen::Vector3d prev_anchor(0, 0, 0);
  const int N = 40;
  for (int k = 0; k < N; ++k) {
    const double s = k / (double)N;
    const double ang = s * 2.0 * M_PI;
    const Eigen::Vector3d t(1.5 * std::cos(ang), 1.5 * std::sin(ang), 0.05 * k);
    slamko::SE3 T(slamko::SO3::exp(Eigen::Vector3d(0, 0, ang)), t);

    viz.setTime((std::uint64_t)k, 0.1 * k);
    // Window A: a synthetic gradient image + moving keypoints.
    cv::Mat img(480, 752, CV_8UC1);
    for (int y = 0; y < img.rows; ++y)
      for (int x = 0; x < img.cols; ++x)
        img.at<uint8_t>(y, x) = (uint8_t)((x + y + k * 8) & 0xFF);
    viz.logImage(img);
    Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> uv(50, 2);
    for (int i = 0; i < 50; ++i) {
      uv(i, 0) = 100 + (i * 13 + k * 5) % 552;
      uv(i, 1) = 80 + (i * 7 + k * 3) % 320;
    }
    const bool loss = (k > 18 && k < 24);  // a fake tracking-loss stretch
    viz.logKeypoints(uv, loss ? slamko::VizTrack::Degraded : slamko::VizTrack::Tracked);
    viz.logPose(T, /*certain=*/k > 30);
    viz.logCamera(T, 380, 380, 376, 240, 752, 480);
    char hud[128];
    std::snprintf(hud, sizeof(hud), "%s | kf %d | loops %d", loss ? "TRACK-LOSS" : "MAPPING",
                  k, k > 35 ? 1 : 0);
    viz.logHud(hud);
    viz.logScalar("keypoints", 50);
    viz.logScalar("loops", k > 35 ? 1 : 0);

    // Seal a submap every 10 KF: a small cloud + a typed chain/soft edge.
    if (k % 10 == 9) {
      std::vector<Eigen::Vector3d> cloud;
      for (int i = 0; i < 200; ++i)
        cloud.push_back(t + Eigen::Vector3d(0.3 * std::cos(i), 0.3 * std::sin(i), 0.4 * (i % 3)));
      const std::uint64_t sid = (std::uint64_t)(k / 10);
      viz.logSubmapCloud(sid, cloud, /*dangling=*/loss);
      (loss ? soft : chain).push_back({prev_anchor, t});
      prev_anchor = t;
    }
    if (k == 38) loop.push_back({Eigen::Vector3d(1.5, 0, 0), t});      // a fake loop
    if (k == 32) prioredge.push_back({Eigen::Vector3d(0.5, 0.5, 0), t});  // a fake x-session prior
    if (k == 28) cand.push_back({Eigen::Vector3d(-0.5, 0.5, 0), t});      // a fake proximity candidate

    viz.setEdges(slamko::VizEdge::Chain, chain);
    viz.setEdges(slamko::VizEdge::Soft, soft);
    viz.setEdges(slamko::VizEdge::Loop, loop);
    viz.setEdges(slamko::VizEdge::Prior, prioredge);
    viz.setEdges(slamko::VizEdge::Candidate, cand);
  }
  std::printf("viz_selftest: wrote %s (open with `rerun %s`)\n", out.c_str(), out.c_str());
  return 0;
}
