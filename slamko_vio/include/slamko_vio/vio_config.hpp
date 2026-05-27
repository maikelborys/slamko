// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// VioConfig — the full parameter set of the VIO pipeline, one plain struct.
// Field names match the ROS param names 1:1 so the node fills them from
// declare_parameter and the (ROS-free) VioPipeline ctor reads cfg.<name>.
// This is what lets VioPipeline be ROS-agnostic (bag/sim/real-portable + unit-
// testable) while the node stays thin I/O glue. Defaults reproduce klt_vo's
// validated baseline.

#pragma once

#include <string>

namespace slamko_vio {

struct VioConfig {
  // image / front-end
  int    image_width        = 752;
  int    image_height       = 480;
  int    max_corners        = 1500;
  int    redetect_threshold = 1500;
  double dedup_radius_px    = 5.0;
  int    patch_size         = 9;
  int    pyramid_levels     = 4;
  std::string timing_csv_path;
  std::string odom_frame_id  = "slamko_vio_world";
  std::string child_frame_id = "slamko_vio_cam";
  bool   publish_tf          = true;
  std::string landmark_dump_path;

  // Shi-Tomasi grid
  int    grid_cols  = 8;
  int    grid_rows  = 6;
  int    k_per_cell = 32;

  // stereo matcher
  int    stereo_patch_size = 11;
  int    min_disparity     = 1;
  int    max_disparity     = 100;
  double stereo_ncc_thr    = 0.6;
  int    stereo_border     = 12;

  // PnP
  double pnp_reproj_thr_px       = 0.8;
  int    pnp_max_iters           = 200;
  int    pnp_min_inliers         = 12;
  double pnp_refine_thr_px       = 0.8;
  bool   pnp_use_cuda            = false;
  int    pnp_cuda_max_points     = 4096;
  int    pnp_lm_max_iters        = 5;
  bool   pnp_refine_second_pass  = true;
  bool   pnp_use_guess           = false;

  // local BA + IMU
  int    ba_window_size          = 10;
  double ba_huber_px             = 3.0;
  int    ba_max_iters            = 30;
  double ba_function_tol         = 1.0e-6;
  int    ba_min_obs_per_landmark = 2;
  bool   enable_imu              = true;
  bool   ba_use_inv_depth        = true;
  double imu_bias_rw_gyro        = 1.9393e-5;
  double imu_bias_rw_accel       = 3.0e-3;
  double imu_gyro_noise_density  = 1.6968e-4;
  double imu_accel_noise_density = 2.0e-3;
  double imu_rate_hz             = 200.0;
  int    imu_init_warmup_samples = 80;

  // dead-reckoning (Workstream R)
  bool   dr_enabled            = false;
  double dr_max_s              = 1.0;
  double dr_force_loss_start_s = -1.0;
  double dr_force_loss_end_s   = -1.0;

  // keyframe triggers
  double kf_translation_m = 0.15;
  double kf_rotation_rad  = 0.087;
  double kf_inlier_drop   = 0.7;
  double kf_parallax_px   = 15.0;
  int    kf_max_frames    = 30;
  int    mature_obs_thr   = 1000;

  // depth gates
  double min_depth_m = 0.3;
  double max_depth_m = 30.0;

  // local map projection (LMP)
  bool   lmp_enabled           = true;
  int    lmp_trigger_active    = 800;
  double lmp_snap_radius_px    = 3.0;
  double lmp_min_depth_m       = 0.3;
  double lmp_max_depth_m       = 25.0;
  int    lmp_max_added_per_frame = 400;
  int    lmp_image_border_px   = 12;
  double lmp_ncc_threshold     = 0.70;

  // feature front-end selector (B2): "shitomasi" | "xfeat"
  std::string feature_source = "shitomasi";
  // XFeat-TRT (used only when feature_source == "xfeat")
  std::string xfeat_onnx_path;                                   // node fills from share/
  std::string xfeat_engine_path = "/tmp/slamko_vio_xfeat_752x480.engine";
  double      xfeat_keypoint_threshold = 0.05;
};

}  // namespace slamko_vio
