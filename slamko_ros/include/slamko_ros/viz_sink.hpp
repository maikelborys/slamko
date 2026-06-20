// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// VizSink — the LIVE debug visualizer sink (Rerun, rerun.io). This is the
// "Pangolin, but modern" surface the offline Plotly scripts can't give: a single
// scrubbable scene-graph with TWO coherent views that the SLAM node feeds in
// real time —
//   Window A ("landmarks over video"): the live left image with XFeat keypoints
//     overlaid (green=tracked / red=degraded) + a HUD/status line + scalar plots.
//   Window B ("map building"): the 3D landmark cloud per submap, the camera pose
//     trajectory growing live, keyframe frustums, and the pose-graph edges drawn
//     BY TYPE (chain / soft-loss-bridged / intra-loop / cross-session-prior /
//     proximity-CANDIDATE) — so soft/uncertain links are visually distinct from
//     the metrically-trusted ones WITHOUT contaminating the map (the research
//     verdict: encode trust by colour + a toggleable entity layer, never a
//     z-offset). See docs/RESEARCH_LIFELONG_FUSION_01.md §viz.
//
// Atlas coherence: everything session-side is logged in the SESSION frame under
// the `world/session` entity, which carries a Transform3D = T_global_session
// (identity until cross-session-localized). When a re-anchor updates that
// transform, the whole session subtree moves onto the prior map coherently — the
// dangling-island Atlas model, rendered honestly. The prior map lives under
// `world/prior` (already global).
//
// Compiled as a complete NO-OP unless the build defines SLAMKO_WITH_RERUN (the
// `SLAMKO_WITH_RERUN` CMake option fetches the Rerun C++ SDK). Every method is
// safe to call when disabled — the node never #includes any Rerun header (PIMPL),
// so it builds with or without the SDK. Single-threaded use only (the node runs a
// single-threaded executor; no locking inside). Connection is connect_grpc() to a
// SEPARATELY-launched `rerun` viewer — never spawn() in-process: that keeps the
// viewer's GPU context + crash handlers out of the estimator process (the OKVIS
// GPU-contention discipline — the viz must never destabilise the estimator).

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "slamko_core/se3.hpp"

namespace slamko {

// Edge classes (the trust ladder). Colour/opacity encodes trust; a separate entity
// path per class gives the viewer a one-click hide toggle (the research's answer to
// "draw soft edges on another plane so they don't contaminate the map").
enum class VizEdge : int {
  Chain        = 0,  // stiff provider/odom edge (blue, solid)          — trusted
  Soft         = 1,  // loss-bridged, dead-reckoned (orange, faint)     — yields under a weld
  Loop         = 2,  // same-session verified loop (red, solid)         — trusted
  Prior        = 3,  // cross-session prior factor / weld (green, solid)— authority
  Candidate    = 4,  // proximity match, pre-promotion (grey, faint)    — NOT in the graph yet
};

// Keypoint/track state -> overlay colour in window A.
enum class VizTrack : int { Tracked = 0, Degraded = 1, Lost = 2 };

class VizSink {
 public:
  VizSink();
  ~VizSink();
  VizSink(const VizSink&) = delete;
  VizSink& operator=(const VizSink&) = delete;

  // Connect to a running `rerun` viewer. `endpoint` empty -> the SDK default
  // (rerun+http://127.0.0.1:9876/proxy). Returns true iff live streaming is on.
  // The no-op build always returns false. Safe to call once at startup.
  bool init(const std::string& app_id, const std::string& endpoint);
  bool enabled() const;

  // Set the scrub timeline BEFORE logging a keyframe's data so window A and window
  // B rewind together. kf_id = the keyframe sequence, stamp = its time [s].
  void setTime(std::uint64_t kf_id, double stamp_s);

  // ---------------------------------------------------------- Window A: over-video
  // Full left image (mono8/8UC1 or 8UC3). Logged to world/session/cam/image.
  void logImage(const cv::Mat& image);
  // Keypoints in FULL-image pixels, coloured by track state. Logged as Points2D
  // pinned to the image entity so they overlay the frame.
  void logKeypoints(const Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>& uv,
                    VizTrack state);
  void logHud(const std::string& line);            // one status line (TextLog)
  void logScalar(const std::string& name, double value);  // plots/<name>

  // ---------------------------------------------------------- Window B: map build
  // session->global transform on the `world/session` subtree (the Atlas coherence
  // hinge). Call whenever T_global_map_ changes (and once at startup with identity).
  void setSessionTransform(const SE3& T_global_session, bool localized);
  // Append the current body pose to the live trajectory (session frame). `certain`
  // -> green (verified-against-prior / freshly loop-closed), else red (dangling/DR).
  void logPose(const SE3& T_session_body, bool certain);
  // Current camera frustum (Pinhole) at world/session/cam, in the session frame.
  void logCamera(const SE3& T_session_cam, double fx, double fy, double cx,
                 double cy, int w, int h);
  // A sealed submap's landmark cloud, in the SESSION frame (sm.anchor*lm already
  // applied by the caller). Logged once per id. `dangling` -> a distinct hue so a
  // hanging island reads as uncertain, never as a fake-coherent extension.
  void logSubmapCloud(std::uint64_t sm_id,
                      const std::vector<Eigen::Vector3d>& session_pts, bool dangling);
  // The prior map's cloud (already in global frame) under world/prior — grey.
  void logPriorCloud(std::uint64_t sm_id, const std::vector<Eigen::Vector3d>& global_pts);
  // REPLACE all edges of one class with these segments (session frame, endpoint
  // pairs). Rerun log semantics are replace-per-entity, so the node rebuilds the
  // full per-class segment list from the graph on each event and calls this.
  void setEdges(VizEdge cls,
                const std::vector<std::array<Eigen::Vector3d, 2>>& segments);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace slamko
