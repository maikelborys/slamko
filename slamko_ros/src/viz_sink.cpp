// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// VizSink implementation. Two builds in one file:
//   * SLAMKO_WITH_RERUN undefined -> every method is a no-op (Impl is empty). The
//     node links this and the viz simply does nothing. No Rerun dependency.
//   * SLAMKO_WITH_RERUN defined    -> Impl wraps a rerun::RecordingStream connected
//     (connect_grpc) to a separately-launched viewer. See viz_sink.hpp for the why.
// If the Rerun C++ API drifts across SDK versions, ALL the churn is contained here.

#include "slamko_ros/viz_sink.hpp"

#ifdef SLAMKO_WITH_RERUN
#include <cstdint>
#include <rerun.hpp>

namespace slamko {
namespace {

// SE3 -> Rerun translation + quaternion (xyzw).
inline rerun::Vec3D vec3(const Eigen::Vector3d& p) {
  return rerun::Vec3D{(float)p.x(), (float)p.y(), (float)p.z()};
}
inline rerun::Quaternion quat(const SE3& T) {
  const Eigen::Quaterniond q = T.so3().unit_quaternion();
  return rerun::Quaternion::from_xyzw((float)q.x(), (float)q.y(), (float)q.z(),
                                      (float)q.w());
}
inline rerun::Transform3D xform(const SE3& T) {
  return rerun::Transform3D(vec3(T.translation()), quat(T));
}

// Trust-ladder colours (RTAB-Map-style: trust by colour, alpha by uncertainty).
uint32_t edgeColor(VizEdge c) {
  switch (c) {
    case VizEdge::Chain:     return 0x4060FFFF;  // blue, solid  — trusted odom/chain
    case VizEdge::Soft:      return 0xFF8C0099;  // orange, faint — dead-reckoned, yields
    case VizEdge::Loop:      return 0xE03030FF;  // red, solid   — same-session loop
    case VizEdge::Prior:     return 0x1ABE1AFF;  // green, solid — cross-session authority
    case VizEdge::Candidate: return 0x9090907F;  // grey, faint  — NOT in the graph yet
  }
  return 0xFFFFFFFF;
}
const char* edgePath(VizEdge c) {
  switch (c) {
    case VizEdge::Chain:     return "world/session/edges/chain";
    case VizEdge::Soft:      return "world/session/edges/soft";
    case VizEdge::Loop:      return "world/session/edges/loop";
    case VizEdge::Prior:     return "world/session/edges/prior";
    case VizEdge::Candidate: return "world/session/edges/candidate";
  }
  return "world/session/edges/other";
}
uint32_t trackColor(VizTrack s) {
  switch (s) {
    case VizTrack::Tracked:  return 0x30E030FF;  // green
    case VizTrack::Degraded: return 0xFFA000FF;  // orange
    case VizTrack::Lost:     return 0xE03030FF;  // red
  }
  return 0xFFFFFFFF;
}
// Stable per-submap hue (golden-ratio walk over HSV-ish RGB).
uint32_t submapColor(std::uint64_t id, bool dangling) {
  if (dangling) return 0xB050D0FF;  // purple = a hanging island (uncertain), never blends in
  const uint32_t lut[10] = {0x4C72B0FF, 0xDD8452FF, 0x55A868FF, 0xC44E52FF,
                            0x8172B3FF, 0x937860FF, 0xDA8BC3FF, 0x8C8C8CFF,
                            0xCCB974FF, 0x64B5CDFF};
  return lut[id % 10];
}

}  // namespace

struct VizSink::Impl {
  std::unique_ptr<rerun::RecordingStream> rec;
  bool on = false;
  // The growing trajectory (re-logged as a connected polyline each pose so the PATH
  // is visible at any scrub time, not just a single moving dot).
  std::vector<rerun::Vec3D> traj;
  std::vector<bool> cert;
};

VizSink::VizSink() : impl_(std::make_unique<Impl>()) {}
VizSink::~VizSink() = default;

bool VizSink::init(const std::string& app_id, const std::string& endpoint) {
  impl_->rec = std::make_unique<rerun::RecordingStream>(
      app_id.empty() ? "slamko" : app_id);
  // endpoint ending in ".rrd" -> record to a FILE (offline, rewindable: open later with
  // `rerun file.rrd` and scrub the whole run). Otherwise connect_grpc to a LIVE viewer
  // (empty = SDK default endpoint). Either way the estimator process never hosts the
  // viewer (no spawn()).
  const bool to_file = endpoint.size() > 4 &&
                       endpoint.compare(endpoint.size() - 4, 4, ".rrd") == 0;
  const auto err = to_file ? impl_->rec->save(endpoint)
                           : (endpoint.empty() ? impl_->rec->connect_grpc()
                                               : impl_->rec->connect_grpc(endpoint));
  impl_->on = err.is_ok();
  if (impl_->on) {
    // A static up axis so window B opens sensibly (REP-103: z-up world).
    impl_->rec->log_static("world",
                           rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
  }
  return impl_->on;
}
bool VizSink::enabled() const { return impl_->on; }

void VizSink::setTime(std::uint64_t kf_id, double stamp_s) {
  if (!impl_->on) return;
  impl_->rec->set_time_sequence("kf", (int64_t)kf_id);
  impl_->rec->set_time_duration_secs("stamp", stamp_s);
}

void VizSink::logImage(const cv::Mat& image) {
  if (!impl_->on || image.empty()) return;
  cv::Mat m = image.isContinuous() ? image : image.clone();
  const uint32_t w = (uint32_t)m.cols, h = (uint32_t)m.rows;
  if (m.channels() == 1) {
    rerun::Collection<uint8_t> blob(
        std::vector<uint8_t>(m.data, m.data + (size_t)w * h));
    impl_->rec->log("world/session/cam/image",
                    rerun::Image::from_grayscale8(std::move(blob), {w, h}));
  } else if (m.channels() == 3) {
    rerun::Collection<uint8_t> blob(
        std::vector<uint8_t>(m.data, m.data + (size_t)w * h * 3));
    impl_->rec->log("world/session/cam/image",
                    rerun::Image::from_rgb24(std::move(blob), {w, h}));
  }
}

void VizSink::logKeypoints(
    const Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>& uv,
    VizTrack state) {
  if (!impl_->on || uv.rows() == 0) return;
  std::vector<rerun::Vec2D> pts;
  pts.reserve(uv.rows());
  for (int i = 0; i < uv.rows(); ++i) pts.push_back({uv(i, 0), uv(i, 1)});
  impl_->rec->log("world/session/cam/image/kpts",
                  rerun::Points2D(pts)
                      .with_colors(rerun::Color(trackColor(state)))
                      .with_radii(2.0f));
}

void VizSink::logHud(const std::string& line) {
  if (!impl_->on) return;
  impl_->rec->log("hud", rerun::TextLog(line));
}
void VizSink::logScalar(const std::string& name, double value) {
  if (!impl_->on) return;
  impl_->rec->log("plots/" + name, rerun::Scalars(value));
}

void VizSink::setSessionTransform(const SE3& T_global_session, bool /*localized*/) {
  if (!impl_->on) return;
  impl_->rec->log("world/session", xform(T_global_session));
}

void VizSink::logPose(const SE3& T_session_body, bool certain) {
  if (!impl_->on) return;
  impl_->traj.push_back(vec3(T_session_body.translation()));
  impl_->cert.push_back(certain);
  // THROTTLE the re-log: the path is re-logged whole each time (Rerun replace semantics),
  // which is O(N) per pose -> O(N²) over a run and chokes the viewer. Re-log only every
  // 8 poses (and always on the very first) — visually identical for a debug view, ~8× less
  // work. (The single new point is cheap to accumulate every pose; only the LOG is heavy.)
  if (impl_->traj.size() > 1 && impl_->traj.size() % 8 != 0) return;
  // The whole path as ONE connected polyline (light blue) — re-logged so the trajectory is
  // a visible growing LINE, not a lone moving dot.
  std::vector<std::vector<rerun::Vec3D>> strip{impl_->traj};
  impl_->rec->log("world/session/traj/line",
                  rerun::LineStrips3D(strip)
                      .with_colors(rerun::Color(0x66B2FFFF))
                      .with_radii(0.01f));
  // The vertices coloured by certainty (green=verified-vs-prior / red=dangling-DR).
  std::vector<rerun::Vec3D> cp, dp;
  for (std::size_t i = 0; i < impl_->traj.size(); ++i)
    (impl_->cert[i] ? cp : dp).push_back(impl_->traj[i]);
  if (!cp.empty())
    impl_->rec->log("world/session/traj/certain",
                    rerun::Points3D(cp).with_colors(rerun::Color(0x1ABE1AFF))
                        .with_radii(0.03f));
  if (!dp.empty())
    impl_->rec->log("world/session/traj/dangling",
                    rerun::Points3D(dp).with_colors(rerun::Color(0xE03030FF))
                        .with_radii(0.03f));
}

void VizSink::logCamera(const SE3& T_session_cam, double fx, double fy, double /*cx*/,
                        double /*cy*/, int w, int h) {
  if (!impl_->on) return;
  // Debug frustum: focal-length + resolution (principal point centred — the few-px
  // offset is visually negligible, and the 2D keypoint overlay is image-space, not
  // governed by this projection).
  impl_->rec->log("world/session/cam", xform(T_session_cam));
  impl_->rec->log("world/session/cam",
                  rerun::Pinhole::from_focal_length_and_resolution(
                      {(float)fx, (float)fy}, {(float)w, (float)h}));
}

void VizSink::logSubmapCloud(std::uint64_t sm_id,
                             const std::vector<Eigen::Vector3d>& session_pts,
                             bool dangling) {
  if (!impl_->on || session_pts.empty()) return;
  std::vector<rerun::Vec3D> pts;
  pts.reserve(session_pts.size());
  for (const auto& p : session_pts) pts.push_back(vec3(p));
  impl_->rec->log("world/session/submap_" + std::to_string(sm_id),
                  rerun::Points3D(pts)
                      .with_colors(rerun::Color(submapColor(sm_id, dangling)))
                      .with_radii(0.015f));
}

void VizSink::logPriorCloud(std::uint64_t sm_id,
                            const std::vector<Eigen::Vector3d>& global_pts) {
  if (!impl_->on || global_pts.empty()) return;
  std::vector<rerun::Vec3D> pts;
  pts.reserve(global_pts.size());
  for (const auto& p : global_pts) pts.push_back(vec3(p));
  impl_->rec->log("world/prior/submap_" + std::to_string(sm_id),
                  rerun::Points3D(pts)
                      .with_colors(rerun::Color(0x777777CC))
                      .with_radii(0.012f));
}

void VizSink::setEdges(VizEdge cls,
                       const std::vector<std::array<Eigen::Vector3d, 2>>& segments) {
  if (!impl_->on) return;
  std::vector<std::vector<rerun::Vec3D>> strips;
  strips.reserve(segments.size());
  for (const auto& s : segments) strips.push_back({vec3(s[0]), vec3(s[1])});
  impl_->rec->log(edgePath(cls),
                  rerun::LineStrips3D(strips)
                      .with_colors(rerun::Color(edgeColor(cls)))
                      .with_radii(0.006f));
}

}  // namespace slamko

#else  // ---------------------------------------------------------- no-op build

namespace slamko {
struct VizSink::Impl {};
VizSink::VizSink() : impl_(nullptr) {}
VizSink::~VizSink() = default;
bool VizSink::init(const std::string&, const std::string&) { return false; }
bool VizSink::enabled() const { return false; }
void VizSink::setTime(std::uint64_t, double) {}
void VizSink::logImage(const cv::Mat&) {}
void VizSink::logKeypoints(
    const Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>&, VizTrack) {}
void VizSink::logHud(const std::string&) {}
void VizSink::logScalar(const std::string&, double) {}
void VizSink::setSessionTransform(const SE3&, bool) {}
void VizSink::logPose(const SE3&, bool) {}
void VizSink::logCamera(const SE3&, double, double, double, double, int, int) {}
void VizSink::logSubmapCloud(std::uint64_t, const std::vector<Eigen::Vector3d>&, bool) {}
void VizSink::logPriorCloud(std::uint64_t, const std::vector<Eigen::Vector3d>&) {}
void VizSink::setEdges(VizEdge, const std::vector<std::array<Eigen::Vector3d, 2>>&) {}
}  // namespace slamko

#endif
