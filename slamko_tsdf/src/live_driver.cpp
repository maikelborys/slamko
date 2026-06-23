// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys

#include "slamko_tsdf/live_driver.hpp"

#include <algorithm>
#include <unordered_set>
#include <utility>

namespace slamko {

VolumetricLiveDriver::VolumetricLiveDriver(
    std::unique_ptr<VolumetricBackend> backend, VolumetricParams vp, LiveParams lp)
    : mapper_(std::move(backend), vp), lp_(lp) {}

bool VolumetricLiveDriver::moved(const SE3& last, const SE3& now) const {
  const double dt = (now.translation() - last.translation()).norm();
  if (dt > lp_.move_threshold_m) return true;
  const double da = (last.so3().inverse() * now.so3()).log().norm();
  return da > lp_.move_threshold_rad;
}

bool VolumetricLiveDriver::addKeyframe(DepthFrame frame, const SE3& world_pose) {
  const std::uint64_t kf = frame.kf_id;
  if (!mapper_.integrateLive(std::move(frame), world_pose)) return false;
  // First time we've seen this kf id → it joins the recency order.
  if (!integrated_pose_.count(kf)) kf_order_.push_back(kf);
  integrated_pose_[kf] = world_pose;
  return true;
}

std::size_t VolumetricLiveDriver::applyCorrection(
    const std::unordered_map<std::uint64_t, SE3>& world_poses) {
  // Which of OUR held keyframes moved past threshold since they were last fused.
  std::vector<std::uint64_t> moved_kfs;
  for (const auto& [kf, last] : integrated_pose_) {
    const auto it = world_poses.find(kf);
    if (it != world_poses.end() && moved(last, it->second))
      moved_kfs.push_back(kf);
  }

  std::size_t refused = 0;
  if (!moved_kfs.empty())
    refused = mapper_.reintegrateWindow(moved_kfs, world_poses);

  // Refresh the diff baseline ONLY for the kfs we actually re-integrated (the
  // moved set). A non-moved kf keeps its last-INTEGRATION pose as the baseline, so
  // a slow sub-threshold creep accumulates and eventually fires — refreshing every
  // snapshot would perpetually reset the creep and never re-integrate it.
  for (std::uint64_t kf : moved_kfs) {
    const auto it = world_poses.find(kf);
    if (it != world_poses.end()) integrated_pose_[kf] = it->second;
  }
  return refused;
}

std::size_t VolumetricLiveDriver::enforceBudget() {
  if (lp_.store_budget_bytes == 0) return 0;
  // The most-recent keep_recent keyframes are the active window — never sealed.
  std::unordered_set<std::uint64_t> keep;
  const std::size_t n = kf_order_.size();
  const std::size_t k = std::min(lp_.keep_recent, n);
  for (std::size_t i = n - k; i < n; ++i) keep.insert(kf_order_[i]);

  std::size_t sealed = 0;
  // Seal oldest-first until under budget (or nothing left outside the window).
  for (std::size_t i = 0; i < n && mapper_.storeBytes() > lp_.store_budget_bytes;
       ++i) {
    const std::uint64_t kf = kf_order_[i];
    if (keep.count(kf)) continue;
    if (mapper_.sealFrame(kf)) ++sealed;
  }
  return sealed;
}

}  // namespace slamko
