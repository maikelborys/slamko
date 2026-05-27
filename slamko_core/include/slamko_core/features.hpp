// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Features — the output contract every detector converges on (validated across
// klt_vo Shi-Tomasi, AirSLAM XFeat-TRT, orbslam3 LiftFeat-m1): keypoints
// (x, y, score) + an optional N×D descriptor block (D=64 for XFeat/LiftFeat,
// L2-normalized float). Detectors with no descriptor (Shi-Tomasi) leave it
// empty; that's a legal Features, the tracker just uses KLT flow.

#pragma once

#include <cstdint>

#include <Eigen/Core>

namespace slamko {

enum class TrackStatus : std::int8_t { Lost = 0, Ok = 1 };

// One persistent feature track across frames (KLT-flow or descriptor-match).
struct Track {
  std::uint32_t id = 0;
  float x = 0.0f;
  float y = 0.0f;
  float score = 0.0f;
  std::uint32_t age = 0;
  TrackStatus status = TrackStatus::Ok;
};

// A frame's detected features. keypoints: N×3 (x, y, score). descriptors:
// N×D row-per-keypoint (empty if the source is descriptor-less). Row i of
// descriptors corresponds to row i of keypoints.
struct Features {
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> keypoints;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> descriptors;

  // Optional global place-recognition (VPR) descriptor for this frame (EigenPlaces,
  // 512-D, L2-normalized) — used by the relocalizer for coarse candidate retrieval.
  // Empty when no VPR front-end ran.
  Eigen::VectorXf global_descriptor;

  int size() const { return static_cast<int>(keypoints.rows()); }
  bool hasGlobalDescriptor() const { return global_descriptor.size() > 0; }
  bool hasDescriptors() const { return descriptors.rows() == keypoints.rows() &&
                                       descriptors.cols() > 0; }
  int descriptorDim() const { return static_cast<int>(descriptors.cols()); }
};

}  // namespace slamko
