// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// vio_pipeline.cpp — the ROS-agnostic stereo-inertial VIO core (see
// vio_pipeline.hpp). Behaviour is verbatim klt_vo (the validated baseline); only
// the I/O boundary changed: ROS image msgs → ImageView, the detector runs through
// slamko_core::FeatureSource (swappable), TF-resolved T_BS arrives via
// setExtrinsics(), and odom/RViz publishing moved to the thin node. Logging is
// VIO_LOG (stderr) instead of rclcpp.
//
// Coordinate convention: left optical frame (x-right, y-down, z-forward).
// world_pose_ is the camera in the world frame: pose_t = pose_{t-1} *
// T_{cur->prev} = pose_{t-1} * inv(T_{prev->cur}).

#include "slamko_vio/vio_pipeline.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <unordered_set>
#include <algorithm>

#include "slamko_vio/ceres_local_smoother.hpp"
#include "slamko_vio/feature/eigenplaces.h"
#include "slamko_vio/feature/shitomasi_source.hpp"
#include "slamko_vio/feature/xfeat_source.hpp"

#define VIO_LOG(fmt, ...) std::fprintf(stderr, "[slamko_vio] " fmt "\n", ##__VA_ARGS__)

namespace slamko_vio {

VioPipeline::VioPipeline(const VioConfig& cfg)
    : VioPipeline(cfg, nullptr) {}

VioPipeline::VioPipeline(const VioConfig& cfg,
                         std::unique_ptr<slamko::LocalSmoother> smoother) {
    smoother_ = std::move(smoother);  // null ⇒ default CeresLocalSmoother below
    image_width_  = cfg.image_width;
    image_height_ = cfg.image_height;
    max_corners_  = cfg.max_corners;
    redetect_thr_ = cfg.redetect_threshold;
    dedup_radius_ = cfg.dedup_radius_px;
    patch_size_   = cfg.patch_size;
    pyramid_lvls_ = cfg.pyramid_levels;
    timing_csv_   = cfg.timing_csv_path;
    // Optional: dump the final BA landmark world map (id x y z obs_count) at
    // shutdown for offline viz. Empty = disabled. PLY-friendly CSV.
    landmark_dump_path_ = cfg.landmark_dump_path;

    slamko_vio::ShiTomasiDetector::Config scfg;
    scfg.max_corners = max_corners_;
    scfg.min_quality = 1.0e-4f;
    scfg.nms_radius  = 3;
    scfg.border      = 8;
    scfg.grid_cols   = cfg.grid_cols;
    scfg.grid_rows   = cfg.grid_rows;
    scfg.k_per_cell  = cfg.k_per_cell;
    // The detector is swappable behind slamko_core::FeatureSource. Select the
    // backend by cfg.feature_source ("shitomasi" baseline | "xfeat" TRT). KLT
    // tracking is unchanged either way (the doc-13 "XFeat-detect + KLT-track").
    if (cfg.feature_source == "xfeat") {
      XFeatConfig xc;
      xc.input_width  = image_width_;
      xc.input_height = image_height_;
      xc.max_keypoints = max_corners_;
      xc.keypoint_threshold = (float)cfg.xfeat_keypoint_threshold;
      xc.onnx_file   = cfg.xfeat_onnx_path;
      xc.engine_file = cfg.xfeat_engine_path;
      feature_source_ = std::make_unique<XFeatSource>(xc);
      VIO_LOG("feature_source = xfeat (onnx=%s)", cfg.xfeat_onnx_path.c_str());
    } else {
      feature_source_ = std::make_unique<ShiTomasiSource>(
          image_width_, image_height_, scfg);
      VIO_LOG("feature_source = shitomasi");
    }

    // Global VPR descriptor (EigenPlaces) for loop-closure RETRIEVAL — built only when
    // enabled (cost only when relocalizing). XFeat local descriptors can't recognize a
    // revisited place; this global descriptor can. See docs/PLAN_VPR_RELOC.md.
    if (cfg.enable_vpr && !cfg.vpr_onnx_path.empty()) {
      EigenPlacesConfig vc;
      vc.onnx_file   = cfg.vpr_onnx_path;
      vc.engine_file = cfg.vpr_engine_path;
      vpr_ = std::make_unique<EigenPlaces>(vc);
      if (vpr_->build()) {
        VIO_LOG("VPR = eigenplaces (onnx=%s)", cfg.vpr_onnx_path.c_str());
      } else {
        VIO_LOG("VPR = eigenplaces BUILD FAILED (onnx=%s) — relocalization retrieval off",
                cfg.vpr_onnx_path.c_str());
        vpr_.reset();
      }
    }

    slamko_vio::KltTracker::Config kcfg;
    kcfg.pyramid_levels = pyramid_lvls_;
    kcfg.patch_size     = patch_size_;
    tracker_ = std::make_unique<slamko_vio::KltTracker>(
        image_width_, image_height_, kcfg);

    slamko_vio::StereoMatcher::Config mcfg;
    mcfg.patch_size    = cfg.stereo_patch_size;
    mcfg.min_disparity = cfg.min_disparity;
    mcfg.max_disparity = cfg.max_disparity;
    mcfg.ncc_threshold = (float)cfg.stereo_ncc_thr;
    mcfg.border        = cfg.stereo_border;
    matcher_ = std::make_unique<slamko_vio::StereoMatcher>(
        image_width_, image_height_, mcfg);

    slamko_vio::PoseEstimator::Config pcfg;
    // R4: tighter RANSAC threshold; 1.5 px was ~30× the ~0.02 px stereo
    // sub-pixel noise floor and let mis-tracked points into LM refine.
    pcfg.reprojection_threshold_px = (float)cfg.pnp_reproj_thr_px;
    pcfg.max_ransac_iters          = cfg.pnp_max_iters;
    pcfg.min_inliers               = cfg.pnp_min_inliers;
    // R4 two-pass: after the first Ceres refine, re-prune to this pixel
    // threshold and refine once more. 0 disables. Default to the same value
    // as RANSAC threshold for now.
    pcfg.refine_pixel_threshold    = (float)cfg.pnp_refine_thr_px;
    pcfg.use_cuda_ransac           = cfg.pnp_use_cuda;
    pcfg.cuda_max_points           = cfg.pnp_cuda_max_points;
    // Sweet-spot default empirically: iters=5 with 2-pass refine. On V1_03
    // it beats iters=10 on BOTH axes (ATE 0.568 vs 0.900, fps 279 vs 161);
    // on MH_01 ATE is unchanged. iters>5 just wastes cycles in Ceres
    // convergence checks.
    pcfg.lm_max_iters              = cfg.pnp_lm_max_iters;
    pcfg.refine_second_pass        = cfg.pnp_refine_second_pass;
    pose_estimator_ = std::make_unique<slamko_vio::PoseEstimator>(pcfg);

    slamko_vio::LocalBA::Config bcfg;
    bcfg.window_size        = cfg.ba_window_size;
    // Loss scale for Cauchy (was Huber 1.0 px). OKVIS2-X uses 3.0.
    bcfg.huber_threshold_px = (float)cfg.ba_huber_px;
    bcfg.max_iterations     = cfg.ba_max_iters;
    bcfg.function_tolerance = cfg.ba_function_tol;
    bcfg.min_observations_per_landmark =
        cfg.ba_min_obs_per_landmark;
    // IMU integration: gyro bias is initialised from the visual rotation
    // chain (mean(ω_imu - ω_visual) over the first 15 KF pairs after T_BS
    // and gravity are calibrated). Accel bias starts at 0; BA refines.
    enable_imu_           = cfg.enable_imu;
    // OV2SLAM anchored inverse-depth landmark parameterisation.
    bcfg.use_inv_depth    = cfg.ba_use_inv_depth;
    bcfg.enable_imu       = enable_imu_;
    bcfg.bias_rw_gyro     = cfg.imu_bias_rw_gyro;
    bcfg.bias_rw_accel    = cfg.imu_bias_rw_accel;
    imu_noise_.gyro_noise_density  =
        cfg.imu_gyro_noise_density;
    imu_noise_.accel_noise_density =
        cfg.imu_accel_noise_density;
    imu_noise_.rate_hz =
        cfg.imu_rate_hz;
    imu_init_warmup_samples_ =
        cfg.imu_init_warmup_samples;  // ~0.4 s @ 200Hz
    // Workstream R: IMU dead-reckoning on tracking loss.
    // Default OFF: dead-reckoning is mechanism-complete but open-loop drift is
    // dominated by the init gravity-DIRECTION error (~16° from motion-
    // contaminated calib) → diverges on multi-second gaps. Needs clean gravity-
    // direction init (low-motion-gated / rotation-compensated window) before it
    // is a net win. See docs/13. Enable explicitly to experiment.
    dr_enabled_ = cfg.dr_enabled;
    dr_max_s_   = cfg.dr_max_s;
    dr_force_loss_start_ =
        cfg.dr_force_loss_start_s;
    dr_force_loss_end_ =
        cfg.dr_force_loss_end_s;
    dr_force_loss_windows_ = cfg.dr_force_loss_windows;
    // T_BS (cam-in-body): set when the static tf imu→left_rect arrives.
    bcfg.T_BS = Eigen::Matrix4d::Identity();
    bcfg.gravity_w = Eigen::Vector3d(0.0, 0.0, -9.81);
    ba_cfg_ = bcfg;
    // Tier-2 backend. Default = CeresLocalSmoother wrapping LocalBA (the P0
    // baseline) built from this cfg; an injected backend (P1c gtsam) is used
    // as-is — the pipeline drives it only through the slamko::LocalSmoother
    // contract (setExtrinsics / setImuParams / setStereoCalib / insertKeyframe).
    if (!smoother_) {
      CeresLocalSmootherConfig scfg;
      scfg.ba    = ba_cfg_;
      scfg.noise = imu_noise_;
      smoother_  = std::make_unique<CeresLocalSmoother>(scfg);
    }

    kf_translation_thr_m_ = (float)cfg.kf_translation_m;
    kf_rotation_thr_rad_  = (float)cfg.kf_rotation_rad;  // ~5°
    kf_inlier_drop_ratio_ = (float)cfg.kf_inlier_drop;
    // Fix A (OV2SLAM): median rotation-uncompensated parallax trigger and a
    // frame-count safety net.
    kf_parallax_px_       = (float)cfg.kf_parallax_px;
    kf_max_frames_        = cfg.kf_max_frames;
    // R2 mature-landmark gating: empirically did not improve ATE at thr={1,2,3}
    // and kf_interval={2,5}. Left in the code path; effectively disabled by
    // default with a very high threshold. Re-enable if a different BA strategy
    // (marginalization / persistent gauge) is implemented.
    mature_obs_thr_       = cfg.mature_obs_thr;

    min_depth_m_ = (float)cfg.min_depth_m;
    // Tight max depth: stereo depth uncertainty grows as Z²/(fx·b)·σ_d, so
    // points beyond ~15 m at EuRoC's geometry have several-cm depth noise
    // that destabilises frame-to-frame PnP.
    max_depth_m_ = (float)cfg.max_depth_m;
    // useExtrinsicGuess forces SOLVEPNP_ITERATIVE which proved noisier than
    // EPnP on EuRoC; default false until we revisit.
    use_pnp_guess_ = cfg.pnp_use_guess;

    // -------- Local map projection (ORB-SLAM3-style re-acquisition) ---------
    // When KLT survival drops below `lmp_trigger_active_`, project all
    // landmark_world_ entries that aren't already being tracked into the
    // current camera, snap to the closest fresh Shi-Tomasi corner within
    // `lmp_snap_radius_px_`, and create a StereoTrack with the existing
    // landmark_id + a known initial 3D from the world map. The new track
    // contributes to PnP starting next frame (after KLT has confirmed it).
    lmp_enabled_              = cfg.lmp_enabled;
    lmp_trigger_active_       = cfg.lmp_trigger_active;
    lmp_snap_radius_px_       = (float)cfg.lmp_snap_radius_px;
    lmp_min_depth_m_          = (float)cfg.lmp_min_depth_m;
    lmp_max_depth_m_          = (float)cfg.lmp_max_depth_m;
    lmp_max_added_per_frame_  = cfg.lmp_max_added_per_frame;
    lmp_image_border_px_      = cfg.lmp_image_border_px;
    lmp_ncc_threshold_        = (float)cfg.lmp_ncc_threshold;

    const std::size_t img_bytes = (std::size_t)image_width_ * image_height_;
    cudaMalloc(&d_left_,    img_bytes);
    cudaMalloc(&d_right_,   img_bytes);
    cudaMalloc(&d_prev_xy_, sizeof(float) * 2 * max_corners_);
    cudaMalloc(&d_curr_xy_, sizeof(float) * 2 * max_corners_);
    cudaMalloc(&d_right_xy_, sizeof(float) * 2 * max_corners_);
    cudaMalloc(&d_status_,  sizeof(std::int8_t) * max_corners_);
    cudaMalloc(&d_stereo_status_, sizeof(std::int8_t) * max_corners_);
    cudaEventCreate(&ev_start_);
    cudaEventCreate(&ev_stop_);

    if (!timing_csv_.empty()) {
      csv_out_.open(timing_csv_);
      csv_out_ << "frame,ms_total,ms_klt,ms_stereo,ms_pnp,ms_detect,ms_ba,"
                  "n_tracked,n_active,n_3d_prev,n_pnp_inliers,n_stereo_match,"
                  "n_new,n_total,n_ba_landmarks,ba_solved,"
                  "n_lmp_attempt,n_lmp_promote,ms_lmp\n";
      csv_out_.flush();
    }

    world_pose_.setIdentity();

    VIO_LOG(
                "slamko_vio_node sprint 2 started — max_corners=%d, patch=%d, "
                "pyr_lvls=%d, stereo_disp=[%d,%d], pnp_thr=%.2f",
                max_corners_, patch_size_, pyramid_lvls_,
                mcfg.min_disparity, mcfg.max_disparity,
                pcfg.reprojection_threshold_px);
  }

VioPipeline::~VioPipeline() {
    if (d_left_)    cudaFree(d_left_);
    if (d_right_)   cudaFree(d_right_);
    if (d_prev_xy_) cudaFree(d_prev_xy_);
    if (d_curr_xy_) cudaFree(d_curr_xy_);
    if (d_right_xy_) cudaFree(d_right_xy_);
    if (d_status_)   cudaFree(d_status_);
    if (d_stereo_status_) cudaFree(d_stereo_status_);
    cudaEventDestroy(ev_start_);
    cudaEventDestroy(ev_stop_);
    if (csv_out_.is_open()) csv_out_.close();
    if (!landmark_dump_path_.empty()) dump_landmarks();
    // Report the reloc-map descriptor index built this session (B3).
    const slamko::SubMap sm = buildSubMap();
    int with_desc = 0;
    for (const auto& l : sm.landmarks) if (l.descriptor_row >= 0) ++with_desc;
    VIO_LOG("submap @ shutdown: %zu landmarks, %d with descriptors (index %dx%d)",
            sm.landmarks.size(), with_desc,
            (int)sm.descriptors.rows(), (int)sm.descriptors.cols());
  }

slamko::SubMap VioPipeline::buildSubMap() const {
  slamko::SubMap sm;
  sm.id = 0;  // anchor/id are owned by the never-lost archive, not the VIO
  // Only landmarks created in the CURRENT submap epoch (since the last beginSubmap)
  // — so sealed submaps are disjoint, not cumulative supersets. epoch 0 with no
  // branch ⇒ every landmark qualifies ⇒ identical to the pre-partition behavior.
  auto in_epoch = [&](std::uint32_t lid) {
    auto it = landmark_epoch_.find(lid);
    return it != landmark_epoch_.end() && it->second == submap_epoch_;
  };
  int n_desc = 0;
  for (const auto& kv : landmark_world_)
    if (in_epoch(kv.first) && landmark_descriptors_.count(kv.first)) ++n_desc;
  sm.descriptors.resize(n_desc, 64);
  sm.landmarks.reserve(landmark_world_.size());
  int row = 0;
  for (const auto& [lid, p] : landmark_world_) {
    if (!in_epoch(lid)) continue;
    slamko::MapLandmark lm;
    lm.id = lid;
    lm.position = p;
    auto dit = landmark_descriptors_.find(lid);
    if (dit != landmark_descriptors_.end()) {
      for (int d = 0; d < 64; ++d) sm.descriptors(row, d) = dit->second[d];
      lm.descriptor_row = row++;
    }
    sm.landmarks.push_back(lm);
  }
  // Attach this epoch's keyframe poses + per-KF 2D observations (BA substrate +
  // real-LightGlue input). `kf_obs` is aligned 1:1 with `keyframes` by construction.
  for (const auto& ek : kf_poses_)
    if (ek.epoch == submap_epoch_) {
      sm.keyframes.push_back(ek.kf);
      sm.kf_obs.push_back(ek.obs);
    }
  sm.global_descriptor = current_global_desc_;  // VPR retrieval vector (empty if no VPR)
  return sm;
}

  // Write the final BA world map to a CSV (id,x,y,z,obs) for offline viz.
void VioPipeline::dump_landmarks() const {
    std::ofstream f(landmark_dump_path_);
    if (!f.is_open()) {
      VIO_LOG("landmark dump: cannot open %s",
                  landmark_dump_path_.c_str());
      return;
    }
    f << "id,x,y,z,obs\n";
    for (const auto& [lid, p] : landmark_world_) {
      int obs = 0;
      auto it = landmark_obs_count_.find(lid);
      if (it != landmark_obs_count_.end()) obs = it->second;
      f << lid << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
        << obs << '\n';
    }
    VIO_LOG("landmark dump: %zu landmarks -> %s",
                landmark_world_.size(), landmark_dump_path_.c_str());
  }

  // Extract an 11×11 mono8 patch from the left ImageView around (u, v). Returns
  // true on success (in-bounds), false if too close to border.
bool VioPipeline::lmp_extract_patch(const slamko::ImageView& img,
                         float u, float v,
                         std::array<std::uint8_t, kLmpPatchPx>& out) const {
    const int xi = (int)(u + 0.5f);
    const int yi = (int)(v + 0.5f);
    if (xi < kLmpPatchHalf || xi >= image_width_  - kLmpPatchHalf) return false;
    if (yi < kLmpPatchHalf || yi >= image_height_ - kLmpPatchHalf) return false;
    const std::uint8_t* base = img.data;
    const std::size_t   step = img.step;
    for (int dy = -kLmpPatchHalf; dy <= kLmpPatchHalf; ++dy) {
      const std::uint8_t* row = base + (yi + dy) * step;
      for (int dx = -kLmpPatchHalf; dx <= kLmpPatchHalf; ++dx) {
        out[(dy + kLmpPatchHalf) * kLmpPatchSide + (dx + kLmpPatchHalf)] =
            row[xi + dx];
      }
    }
    return true;
  }

  // Zero-mean normalized cross-correlation between two 11×11 patches. Returns
  // value in [-1, 1]; 0 if either patch has zero variance.
float VioPipeline::lmp_ncc(const std::array<std::uint8_t, kLmpPatchPx>& a,
                       const std::array<std::uint8_t, kLmpPatchPx>& b) {
    float sa = 0.f, sb = 0.f;
    for (int i = 0; i < kLmpPatchPx; ++i) { sa += a[i]; sb += b[i]; }
    const float ma = sa / (float)kLmpPatchPx;
    const float mb = sb / (float)kLmpPatchPx;
    float num = 0.f, da = 0.f, db = 0.f;
    for (int i = 0; i < kLmpPatchPx; ++i) {
      const float ax = (float)a[i] - ma;
      const float bx = (float)b[i] - mb;
      num += ax * bx;
      da  += ax * ax;
      db  += bx * bx;
    }
    if (da < 1e-3f || db < 1e-3f) return 0.f;
    return num / std::sqrt(da * db);
  }

  // Pop and return IMU samples with timestamps in (t_lo, t_hi]. Adds the
  // bracketing samples on each side so the integration spans the exact window.
std::vector<ImuSample> VioPipeline::drain_imu_window(double t_lo, double t_hi) {
    std::vector<ImuSample> out;
    std::lock_guard<std::mutex> lk(imu_mutex_);
    if (imu_buffer_.empty()) return out;
    // Find the first sample at or after t_lo, and last <= t_hi.
    out.reserve(imu_buffer_.size());
    for (const auto& s : imu_buffer_) {
      if (s.t < t_lo - 0.01)  continue;          // discard far-past
      if (s.t > t_hi + 0.01)  break;             // future
      out.push_back(s);
    }
    return out;
  }

void VioPipeline::processStereo(const slamko::ImageView& left,
                                const slamko::ImageView& right,
                                double timestamp, const StereoIntrinsics& K) {
    const bool first_K = !have_K_;
    K_ = K;
    have_K_ = true;
    if (first_K) {
      // Hand the rectified-stereo calib to the Tier-2 backend once. The
      // LocalSmoother contract carries observations WITHOUT K (unlike LocalBA's
      // per-insert K), so the backend must be told the calibration here — both
      // CeresLocalSmoother and GtsamLocalSmoother need it before insertKeyframe.
      smoother_->setStereoCalib(
          slamko::StereoCalib{K.fx, K.fy, K.cx, K.cy, K.baseline_m});
    }
    cudaEventRecord(ev_start_);

    // Global VPR descriptor for this frame (loop-closure retrieval). Wrap the left
    // ImageView as a cv::Mat (no copy) and run EigenPlaces; the node stamps the result
    // onto the reloc query + each submap. Computed only when cfg.enable_vpr built vpr_.
    if (vpr_) {
      const cv::Mat left_mat(left.height, left.width, CV_8UC1,
                             const_cast<std::uint8_t*>(left.data), left.step);
      Eigen::VectorXf g;
      if (vpr_->infer(left_mat, g)) current_global_desc_ = std::move(g);
    }

    // Upload mono8 stereo pair.
    cudaMemcpy2DAsync(d_left_,  image_width_,
                      left.data, left.step,
                      image_width_, image_height_, cudaMemcpyHostToDevice);
    cudaMemcpy2DAsync(d_right_, image_width_,
                      right.data, right.step,
                      image_width_, image_height_, cudaMemcpyHostToDevice);

    tracker_->set_image(d_left_, image_width_);
    matcher_->set_images(d_left_, image_width_, d_right_, image_width_);

    // ------------------------------------------------------------- KLT track
    const int n_prev = (int)tracks_.size();
    int n_active = 0;
    float ms_klt = 0.f;
    if (n_prev > 0 && tracker_has_prev_) {
      std::vector<float> h_prev(2 * n_prev);
      for (int i = 0; i < n_prev; ++i) {
        h_prev[2*i + 0] = tracks_[i].left_curr_x;  // promotes to "prev" this frame
        h_prev[2*i + 1] = tracks_[i].left_curr_y;
      }
      cudaMemcpyAsync(d_prev_xy_, h_prev.data(), sizeof(float) * 2 * n_prev,
                      cudaMemcpyHostToDevice);

      cudaEvent_t a, b; cudaEventCreate(&a); cudaEventCreate(&b);
      cudaEventRecord(a);
      tracker_->track(d_prev_xy_, d_curr_xy_, d_status_, n_prev);
      cudaEventRecord(b);
      cudaEventSynchronize(b);
      cudaEventElapsedTime(&ms_klt, a, b);
      cudaEventDestroy(a); cudaEventDestroy(b);

      std::vector<float>        h_curr(2 * n_prev);
      std::vector<std::int8_t>  h_stat(n_prev);
      cudaMemcpy(h_curr.data(), d_curr_xy_, sizeof(float) * 2 * n_prev,
                 cudaMemcpyDeviceToHost);
      cudaMemcpy(h_stat.data(), d_status_,  sizeof(std::int8_t) * n_prev,
                 cudaMemcpyDeviceToHost);

      std::vector<StereoTrack> kept;
      kept.reserve(n_prev);
      for (int i = 0; i < n_prev; ++i) {
        if (h_stat[i] != 1) continue;
        StereoTrack t = tracks_[i];
        t.left_prev_x = t.left_curr_x;
        t.left_prev_y = t.left_curr_y;
        t.left_curr_x = h_curr[2*i + 0];
        t.left_curr_y = h_curr[2*i + 1];
        t.age += 1;
        kept.push_back(t);
      }
      tracks_ = std::move(kept);
      n_active = (int)tracks_.size();
    }

    // -------------------- Detect new corners (before stereo match + PnP) ----
    int n_new = 0;
    float ms_detect = 0.f;
    if ((int)tracks_.size() < redetect_thr_) {
      // Detection runs through the swappable FeatureSource (ShiTomasi here,
      // XFeat in B2). It owns its device image + upload internally, so we time
      // it on the host clock (the GPU-event wrap no longer brackets the call).
      const auto det_t0 = std::chrono::steady_clock::now();
      slamko::Features det = feature_source_->detect(left);
      const auto det_t1 = std::chrono::steady_clock::now();
      ms_detect = std::chrono::duration<float, std::milli>(det_t1 - det_t0).count();
      const float r2 = (float)(dedup_radius_ * dedup_radius_);
      for (int ki = 0; ki < det.size(); ++ki) {
        const float kx = det.keypoints(ki, 0);
        const float ky = det.keypoints(ki, 1);
        bool dup = false;
        for (const auto& t : tracks_) {
          const float dx = t.left_curr_x - kx;
          const float dy = t.left_curr_y - ky;
          if (dx*dx + dy*dy < r2) { dup = true; break; }
        }
        if (dup) continue;
        StereoTrack nt;
        nt.id = next_id_++;
        nt.left_prev_x = kx;
        nt.left_prev_y = ky;
        nt.left_curr_x = kx;
        nt.left_curr_y = ky;
        nt.has_right_curr = false;
        nt.has_3d_prev    = false;
        nt.has_3d_curr    = false;
        nt.age = 0;
        // Capture the learned descriptor at birth (XFeat: 64-d). Travels with
        // the track through KLT culling; copied to the landmark at KF rate.
        if (det.hasDescriptors() && det.descriptorDim() == 64) {
          for (int d = 0; d < 64; ++d) nt.desc[d] = det.descriptors(ki, d);
          nt.has_desc = true;
        }
        tracks_.push_back(nt);
        ++n_new;
        if ((int)tracks_.size() >= max_corners_) break;
      }
    }

    // -------------------- Local Map Projection (LMP) ------------------------
    // ORB-SLAM3-style re-acquisition. For each landmark in the world map that
    // we are NOT currently tracking, project it into the current camera using
    // the previous frame's pose (T_w_c_, frame N-1). If the projection lands
    // inside the image and a recently-detected Shi-Tomasi corner exists within
    // lmp_snap_radius_px_ of the prediction, promote that corner-track to a
    // recovered LMP track: assign the existing landmark_id and seed
    // point_3d_curr from the world map.
    //
    // Why this works: the new-corner tracks just added above have landmark_id=
    // 0 (no map identity). When a recently-known landmark projects onto one of
    // these corners, it's almost certainly the same physical feature reborn
    // after a KLT loss. The promotion gives the next-frame PnP an instant 3D
    // anchor instead of waiting 2 frames for fresh stereo triangulation, and
    // restores the track's age + landmark history (BA can re-use it).
    int n_lmp_attempt = 0, n_lmp_promote = 0;
    float ms_lmp = 0.f;
    // Trigger: when KLT lost enough tracks (n_active is the post-KLT count;
    // tracks_.size() at this point has been topped up by redetect).
    if (lmp_enabled_ && have_world_pose_ && have_K_ &&
        n_active < lmp_trigger_active_ &&
        !landmark_world_.empty()) {
      const auto lmp_t0 = std::chrono::steady_clock::now();
      // 1. Build a flat list of corner-track indices (those with landmark_id=0
      //    and freshly born this frame — i.e., the new tracks the redetect
      //    block just appended) along with their pixel positions.
      const int Ntr = (int)tracks_.size();
      std::vector<int> cand_idx;
      cand_idx.reserve(Ntr);
      for (int i = 0; i < Ntr; ++i) {
        if (tracks_[i].landmark_id == 0 && tracks_[i].age == 0) {
          cand_idx.push_back(i);
        }
      }
      if (!cand_idx.empty()) {
        // 2. Spatial hash over the candidate corner-tracks. Image is image_width_
        //    × image_height_; choose ~24 px cells. Cell coords: (x/24, y/24).
        const int cell_sz = 24;
        const int gx = (image_width_  + cell_sz - 1) / cell_sz;
        const int gy = (image_height_ + cell_sz - 1) / cell_sz;
        std::vector<std::vector<int>> grid(gx * gy);
        for (int j : cand_idx) {
          const int cx = std::min(std::max((int)(tracks_[j].left_curr_x / cell_sz), 0), gx - 1);
          const int cy = std::min(std::max((int)(tracks_[j].left_curr_y / cell_sz), 0), gy - 1);
          grid[cy * gx + cx].push_back(j);
        }
        // 3. Build "currently-tracked landmark_ids" set so we don't re-add.
        std::unordered_set<std::uint32_t> live_lids;
        live_lids.reserve(Ntr);
        for (const auto& t : tracks_) {
          if (t.landmark_id != 0) live_lids.insert(t.landmark_id);
        }
        // 4. Project & match. T_w_c_ is the world→cam transform at frame N-1
        //    (small motion error absorbed by the snap radius).
        const float fx = (float)K_.fx, fy_ = (float)K_.fy;
        const float cx_ = (float)K_.cx, cy_ = (float)K_.cy;
        const float border = (float)lmp_image_border_px_;
        const float r2 = lmp_snap_radius_px_ * lmp_snap_radius_px_;
        // Track index → "already promoted this frame" flag. O(1) lookup
        // replaces the O(N) std::find that killed V1_03 perf.
        std::vector<char> track_used(Ntr, 0);
        // Fast culling: world-space distance from camera origin to landmark.
        // Camera origin in world = -R^T * t = world_pose_.block<3,1>(0,3).
        const Eigen::Vector3d cam_origin_w =
            world_pose_.block<3, 1>(0, 3).cast<double>();
        const double max_dist2 =
            (double)lmp_max_depth_m_ * (double)lmp_max_depth_m_ * 1.5;  // generous
        for (const auto& kv : landmark_world_) {
          const std::uint32_t lid = kv.first;
          if (live_lids.count(lid)) continue;
          if (n_lmp_promote >= lmp_max_added_per_frame_) break;
          // World-space distance gate (cheap pre-filter — avoids projection).
          const Eigen::Vector3d dw = kv.second - cam_origin_w;
          if (dw.squaredNorm() > max_dist2) continue;
          ++n_lmp_attempt;
          const Eigen::Vector4d p_w(kv.second.x(), kv.second.y(), kv.second.z(), 1.0);
          const Eigen::Vector4d p_c = T_w_c_ * p_w;
          if (p_c.z() < (double)lmp_min_depth_m_ ||
              p_c.z() > (double)lmp_max_depth_m_) continue;
          const float inv_z = 1.f / (float)p_c.z();
          const float u = fx * (float)p_c.x() * inv_z + cx_;
          const float v = fy_ * (float)p_c.y() * inv_z + cy_;
          if (u < border || u > (float)image_width_  - border) continue;
          if (v < border || v > (float)image_height_ - border) continue;
          const int cx_i = std::min(std::max((int)(u / cell_sz), 0), gx - 1);
          const int cy_i = std::min(std::max((int)(v / cell_sz), 0), gy - 1);
          int best_track = -1;
          float best_d2 = r2 + 1.f;
          for (int dy = -1; dy <= 1; ++dy) {
            const int yy = cy_i + dy;
            if (yy < 0 || yy >= gy) continue;
            for (int dx = -1; dx <= 1; ++dx) {
              const int xx = cx_i + dx;
              if (xx < 0 || xx >= gx) continue;
              const auto& bucket = grid[yy * gx + xx];
              for (int ti : bucket) {
                if (track_used[ti]) continue;
                const float du = tracks_[ti].left_curr_x - u;
                const float dv = tracks_[ti].left_curr_y - v;
                const float d2 = du*du + dv*dv;
                if (d2 < best_d2) { best_d2 = d2; best_track = ti; }
              }
            }
          }
          if (best_track < 0) continue;
          // LMP v4: NCC patch verification. Reject snap-to-corner matches
          // where the local appearance differs from the stored landmark
          // patch. Without this, false matches in textured regions
          // regressed MH_05 / V1_01 in v3.
          auto pit = landmark_patch_.find(lid);
          if (pit != landmark_patch_.end()) {
            std::array<std::uint8_t, kLmpPatchPx> cur_patch;
            if (!lmp_extract_patch(left,
                                   tracks_[best_track].left_curr_x,
                                   tracks_[best_track].left_curr_y,
                                   cur_patch)) continue;
            const float ncc = lmp_ncc(pit->second, cur_patch);
            if (ncc < lmp_ncc_threshold_) continue;
          }
          tracks_[best_track].landmark_id  = lid;
          tracks_[best_track].from_lmp     = true;
          tracks_[best_track].point_3d_curr = p_c.head<3>().cast<float>();
          tracks_[best_track].has_3d_curr  = true;
          track_used[best_track] = 1;
          ++n_lmp_promote;
        }
      }
      const auto lmp_t1 = std::chrono::steady_clock::now();
      ms_lmp = std::chrono::duration<float, std::milli>(lmp_t1 - lmp_t0).count();
    }
    (void)n_lmp_attempt; (void)ms_lmp;

    // --------------- Stereo match + triangulate (BEFORE PnP, R1+R5) ---------
    // PnP needs the right-cam pixel in the current frame for both-camera
    // reprojection. The triangulated 3D point is also used as PnP's
    // previous-frame anchor: after PnP we commit point_3d_curr → point_3d_prev
    // for next frame. (R2 — persistent-map PnP — regressed badly in
    // experiments because new landmarks anchor to a possibly-drifting pose
    // at birth and BA refinements desync with the next-frame PnP; deferred.)
    int n_stereo_match = 0;
    float ms_stereo = 0.f;
    if (!tracks_.empty()) {
      const int N = (int)tracks_.size();
      std::vector<float> h_left(2 * N);
      for (int i = 0; i < N; ++i) {
        h_left[2*i + 0] = tracks_[i].left_curr_x;
        h_left[2*i + 1] = tracks_[i].left_curr_y;
      }
      cudaMemcpyAsync(d_curr_xy_, h_left.data(), sizeof(float) * 2 * N,
                      cudaMemcpyHostToDevice);

      cudaEvent_t a, b; cudaEventCreate(&a); cudaEventCreate(&b);
      cudaEventRecord(a);
      matcher_->match(d_curr_xy_, d_right_xy_, d_stereo_status_, N);
      cudaEventRecord(b);
      cudaEventSynchronize(b);
      cudaEventElapsedTime(&ms_stereo, a, b);
      cudaEventDestroy(a); cudaEventDestroy(b);

      std::vector<float>       h_right(2 * N);
      std::vector<std::int8_t> h_smatch(N);
      cudaMemcpy(h_right.data(),  d_right_xy_,       sizeof(float) * 2 * N,
                 cudaMemcpyDeviceToHost);
      cudaMemcpy(h_smatch.data(), d_stereo_status_,  sizeof(std::int8_t) * N,
                 cudaMemcpyDeviceToHost);

      // Cache LMP-derived 3D before the stereo loop resets has_3d_curr; this
      // lets us restore the map-derived anchor for any LMP track that stereo
      // fails to match this frame.
      std::vector<Eigen::Vector3f> lmp_3d_backup(N);
      std::vector<char>           lmp_3d_have(N, 0);
      for (int i = 0; i < N; ++i) {
        if (tracks_[i].from_lmp && tracks_[i].has_3d_curr) {
          lmp_3d_backup[i] = tracks_[i].point_3d_curr;
          lmp_3d_have[i] = 1;
        }
      }
      for (int i = 0; i < N; ++i) {
        tracks_[i].has_right_curr = false;
        tracks_[i].has_3d_curr    = false;
        if (h_smatch[i] != 1) continue;
        const float ux_r = h_right[2*i + 0];
        const float uy_r = h_right[2*i + 1];
        Eigen::Vector3f p3;
        if (slamko_vio::triangulate_stereo(h_left[2*i + 0], ux_r,
                                       h_left[2*i + 1], K_,
                                       min_depth_m_, max_depth_m_, p3)) {
          tracks_[i].right_curr_x   = ux_r;
          tracks_[i].right_curr_y   = uy_r;
          tracks_[i].has_right_curr = true;
          tracks_[i].point_3d_curr  = p3;
          tracks_[i].has_3d_curr    = true;
          ++n_stereo_match;
        }
      }
      // LMP fallback: restore map-derived 3D for LMP tracks where stereo
      // failed. Without this, the LMP recovery contributes nothing in the
      // very conditions (blur, low texture) it was designed to fix.
      for (int i = 0; i < N; ++i) {
        if (lmp_3d_have[i] && !tracks_[i].has_right_curr) {
          tracks_[i].point_3d_curr = lmp_3d_backup[i];
          tracks_[i].has_3d_curr   = true;
        }
      }
    }

    // Frame timestamp + forced-loss window (Workstream R test hook). Computed
    // BEFORE PnP so a forced loss actually SUPPRESSES the PnP pose update
    // (simulating real visual loss) rather than running dead-reckoning on top
    // of a still-working PnP.
    const double frame_ts =
        timestamp;
    if (!have_seq_t0_) { seq_t0_ = frame_ts; have_seq_t0_ = true; }
    const double t_rel = frame_ts - seq_t0_;
    bool forced_loss =
        (dr_force_loss_start_ >= 0.0 && t_rel >= dr_force_loss_start_ &&
         t_rel < dr_force_loss_end_);
    for (const auto& w : dr_force_loss_windows_)
      if (t_rel >= w.first && t_rel < w.second) { forced_loss = true; break; }

    // ------------------------- PnP RANSAC + both-cam Ceres refine (R1 + R2)
    // R2 (mature-landmark gating): each landmark accumulates a count of how
    // many BA keyframes it has appeared in (`landmark_obs_count_`). For
    // tracks whose landmark is mature (count ≥ mature_obs_thr_), we use the
    // BA-refined world position transformed to the previous-frame camera
    // coords as PnP's 3D anchor — this breaks the fresh-triangulation
    // correlated-noise loop. Young tracks fall back to fresh stereo
    // triangulation in cam coords.
    Eigen::Matrix4f T_pp = Eigen::Matrix4f::Identity();
    int n_pnp_in = 0;
    float ms_pnp = 0.f;
    int n_3d_prev = 0;
    int n_mature  = 0;
    bool pnp_ok = false;   // did PnP produce a pose this frame? (dead-reckoning)
    if (!tracks_.empty()) {
      const Eigen::Matrix4d T_w_c_prev = T_w_c_;        // pose BEFORE this PnP
      std::vector<Eigen::Vector3f> p3d;
      std::vector<Eigen::Vector2f> p2d_l, p2d_r;
      std::vector<int> idx_map;
      const int Ntr = (int)tracks_.size();
      p3d.reserve(Ntr); p2d_l.reserve(Ntr); p2d_r.reserve(Ntr);
      idx_map.reserve(Ntr);
      const float nan_f = std::numeric_limits<float>::quiet_NaN();
      for (int i = 0; i < Ntr; ++i) {
        const auto& t = tracks_[i];
        // We need both a current-frame left observation (always true: tracks
        // exist after KLT) and SOME 3D anchor. Prefer mature world landmark.
        bool   used   = false;
        Eigen::Vector3f p_prev;
        if (t.landmark_id != 0) {
          auto cit = landmark_obs_count_.find(t.landmark_id);
          auto wit = landmark_world_.find(t.landmark_id);
          if (cit != landmark_obs_count_.end() && cit->second >= mature_obs_thr_
              && wit != landmark_world_.end()) {
            // Transform refined world → previous-frame cam coords.
            const Eigen::Vector4d p_w(wit->second.x(), wit->second.y(),
                                       wit->second.z(), 1.0);
            const Eigen::Vector4d p_c = T_w_c_prev * p_w;
            if (p_c.z() > min_depth_m_) {
              p_prev = p_c.head<3>().cast<float>();
              used = true;
              ++n_mature;
            }
          }
        }
        if (!used) {
          if (!t.has_3d_prev) continue;
          p_prev = t.point_3d_prev;
        }
        p3d.push_back(p_prev);
        p2d_l.emplace_back(t.left_curr_x, t.left_curr_y);
        p2d_r.emplace_back(t.has_right_curr ? t.right_curr_x : nan_f,
                           t.has_right_curr ? t.right_curr_y : nan_f);
        idx_map.push_back(i);
      }
      n_3d_prev = (int)p3d.size();

      if (n_3d_prev >= 12 && !forced_loss) {
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<int> inliers;
        if (use_pnp_guess_ && have_world_pose_) T_pp = last_T_pp_;
        const bool ok = pose_estimator_->solve(
            p3d, p2d_l, p2d_r, K_, T_pp, inliers,
            /*use_guess=*/use_pnp_guess_ && have_world_pose_);
        pnp_ok = ok;
        if (ok) {
          n_pnp_in = (int)inliers.size();
          world_pose_ = world_pose_ * T_pp.inverse().cast<float>();
          T_w_c_      = world_pose_.cast<double>().inverse();
          last_T_pp_  = T_pp;
          have_world_pose_ = true;

          // Drop tracks rejected as PnP outliers.
          const int n_active_before = (int)tracks_.size();
          std::vector<bool> keep(n_active_before, true);
          for (int i : idx_map) keep[i] = false;
          for (int i : inliers) keep[idx_map[i]] = true;
          std::vector<StereoTrack> kept;
          kept.reserve(n_active_before);
          for (int i = 0; i < n_active_before; ++i)
            if (keep[i]) kept.push_back(tracks_[i]);
          tracks_ = std::move(kept);
          n_active = (int)tracks_.size();
        }
        const auto t1 = std::chrono::steady_clock::now();
        ms_pnp = std::chrono::duration<float, std::milli>(t1 - t0).count();
      }
    }

    // ---------------------------- Workstream R: tracking-loss dead-reckoning
    // If PnP did not produce a pose this frame (too few 3D points, or RANSAC
    // failed), propagate the pose with preintegrated IMU instead of letting it
    // freeze. Bridges short visual gaps (motion blur, low texture, occlusion);
    // re-anchors automatically once PnP recovers. Long gaps diverge (IMU-only),
    // so dr_max_s_ bounds how long we trust it before warning.
    const bool vision_lost = (!pnp_ok) || forced_loss;

    if (dr_enabled_ && vision_lost && enable_imu_ && imu_initialised_ &&
        T_BS_resolved_ && have_world_pose_ && last_frame_ts_ > 0.0) {
      const double dt = frame_ts - last_frame_ts_;
      if (dt > 1.0e-4 && dt < 0.5) {
        auto samples = drain_imu_window(last_frame_ts_, frame_ts);
        if (samples.size() >= 2) {
          slamko_vio::ImuPreintegration pi(bias_lin_, imu_noise_);
          pi.integrate_span(samples.data(), samples.size());
          Eigen::Matrix3d dR; Eigen::Vector3d dV, dP;
          pi.corrected(bias_lin_, dR, dV, dP);
          // Body pose in world: T_WB = T_WC * T_BS^{-1}  (world_pose_ = T_WC).
          const Eigen::Matrix4d T_WB_i =
              world_pose_.cast<double>() * T_BS_.inverse();
          const Eigen::Matrix3d R_WB_i = T_WB_i.block<3, 3>(0, 0);
          const Eigen::Vector3d p_WB_i = T_WB_i.block<3, 1>(0, 3);
          // Constant-velocity + gyro-rotation dead-reckoning (NOT full
          // strapdown). On this lean VI backend the velocity / accel-bias /
          // gravity-direction are jointly fit by BA (great poses, ATE 0.054)
          // but not individually consistent enough to double-integrate the
          // accelerometer open-loop — full Forster strapdown injects ~2.5 m/s²
          // spurious accel and diverges within ~0.2 s. The BA VELOCITY however
          // is accurate (|v|=0.50 vs GT 0.49 at onset), and the gyro (small
          // bias, no gravity coupling) gives reliable rotation. So we hold
          // velocity constant and integrate only the gyro. Bridges short gaps;
          // longer losses are the relocalizer's job. See docs/13.
          const Eigen::Matrix3d R_WB_j = R_WB_i * dR;            // gyro rotation
          const Eigen::Vector3d p_WB_j = p_WB_i + velocity_w_ * dt;  // const vel
          (void)dV; (void)dP;   // accelerometer intentionally NOT integrated
          Eigen::Matrix4d T_WB_j = Eigen::Matrix4d::Identity();
          T_WB_j.block<3, 3>(0, 0) = R_WB_j;
          T_WB_j.block<3, 1>(0, 3) = p_WB_j;
          const Eigen::Matrix4d T_WC_j = T_WB_j * T_BS_;  // body → camera
          world_pose_ = T_WC_j.cast<float>();
          T_w_c_      = T_WC_j.inverse();
          ++dr_frames_;
          if (!in_dead_reckoning_) {
            in_dead_reckoning_ = true;
            dr_start_ts_ = last_frame_ts_;
            VIO_LOG(
                "tracking loss @ t=%.2fs%s — dead-reckoning on IMU "
                "(v_w=[%.3f %.3f %.3f] |v|=%.3f m/s, g_w=[%.2f %.2f %.2f])",
                t_rel, forced_loss ? " (forced)" : "",
                velocity_w_.x(), velocity_w_.y(), velocity_w_.z(),
                velocity_w_.norm(), ba_cfg_.gravity_w.x(),
                ba_cfg_.gravity_w.y(), ba_cfg_.gravity_w.z());
          } else if (frame_ts - dr_start_ts_ > dr_max_s_) {
            VIO_LOG(
                "dead-reckoning %.1fs > dr_max_s — pose increasingly unreliable",
                frame_ts - dr_start_ts_);
          }
        }
      }
    } else if (in_dead_reckoning_ && !vision_lost) {
      VIO_LOG(
          "tracking recovered @ t=%.2fs after %d dead-reckoned frames (%.2fs)",
          t_rel, dr_frames_, frame_ts - dr_start_ts_);
      in_dead_reckoning_ = false;
      dr_frames_ = 0;
    }

    // ---------------------------- Sprint 3: KF insertion + local BA refine
    float ms_ba = 0.f;
    int  n_ba_landmarks = 0;
    bool ba_solved = false;
    ++frames_since_last_kf_;
    bool kf_due = false;

    // Fix A (OV2SLAM-style): median pixel parallax since last KF. Triggers
    // KF only when scene has visually changed enough — handles slow motion
    // (V1_01) without inserting KFs at tiny baselines. Tried rotation-
    // compensated parallax (homography subtract); empirically WORSE on
    // V1_03/MH_03 because the depth-∞ assumption breaks for close points.
    double median_parallax_px = 0.0;
    if (have_last_kf_) {
      std::vector<double> parallaxes;
      parallaxes.reserve(tracks_.size());
      for (const auto& t : tracks_) {
        if (!t.has_at_last_kf) continue;
        const double du = (double)t.left_curr_x - (double)t.left_at_last_kf_x;
        const double dv = (double)t.left_curr_y - (double)t.left_at_last_kf_y;
        parallaxes.push_back(std::hypot(du, dv));
      }
      if (!parallaxes.empty()) {
        const std::size_t mid = parallaxes.size() / 2;
        std::nth_element(parallaxes.begin(), parallaxes.begin() + mid,
                          parallaxes.end());
        median_parallax_px = parallaxes[mid];
      }
    }

    if (!have_last_kf_) {
      kf_due = (n_pnp_in > 0) ? true : (n_stereo_match >= 12);
    } else {
      const Eigen::Matrix4d dT = T_w_c_at_last_kf_ * T_w_c_.inverse();
      const double dt_m = dT.block<3, 1>(0, 3).norm();
      const Eigen::AngleAxisd aa(Eigen::Matrix3d(dT.block<3, 3>(0, 0)));
      const double dr_rad = std::abs(aa.angle());
      const bool inliers_dropped =
          (first_kf_inliers_ > 0) &&
          (n_pnp_in < (int)(kf_inlier_drop_ratio_ * (float)first_kf_inliers_));
      // Parallax-triggered KF is the primary policy. Best mean ATE across
      // the EuRoC suite vs frame-interval or combined-OR triggers. Frame
      // safety net (30 frames) handles static-camera segments.
      const bool parallax_trigger = median_parallax_px >= kf_parallax_px_;
      const bool frame_safety     = frames_since_last_kf_ >= kf_max_frames_;
      kf_due = parallax_trigger || frame_safety ||
               (dt_m > kf_translation_thr_m_) ||
               (dr_rad > kf_rotation_thr_rad_) || inliers_dropped;
    }

    if (kf_due && n_stereo_match >= 12 && have_K_ && !forced_loss) {
      std::vector<std::uint32_t>      lids;
      std::vector<Eigen::Vector2d>    uvs_l, uvs_r;
      std::vector<Eigen::Vector3d>    wps;
      lids.reserve(n_stereo_match);
      uvs_l.reserve(n_stereo_match);
      uvs_r.reserve(n_stereo_match);
      wps.reserve(n_stereo_match);
      const double nan_d = std::numeric_limits<double>::quiet_NaN();
      const Eigen::Matrix4d T_c_w = T_w_c_.inverse();   // cam-to-world
      for (auto& t : tracks_) {
        if (!t.has_3d_curr) continue;
        if (t.landmark_id == 0) {
          t.landmark_id = next_lid_++;
          const Eigen::Vector3d p_cam = t.point_3d_curr.cast<double>();
          const Eigen::Vector4d p_w = T_c_w * p_cam.homogeneous();
          landmark_world_[t.landmark_id] = p_w.head<3>();
          landmark_epoch_[t.landmark_id] = submap_epoch_;  // owns it for this submap
          // B3: stamp the landmark with the track's birth descriptor → reloc map.
          if (t.has_desc) landmark_descriptors_[t.landmark_id] = t.desc;
          // LMP v4: cache the appearance patch so re-acquisition can verify
          // snap-to-corner matches via NCC. Done only at first landmark
          // observation — the appearance is most reliable from the moment
          // we triangulated the 3D position.
          std::array<std::uint8_t, kLmpPatchPx> patch;
          if (lmp_extract_patch(left, t.left_curr_x, t.left_curr_y, patch)) {
            landmark_patch_[t.landmark_id] = patch;
          }
        }
        lids.push_back(t.landmark_id);
        uvs_l.emplace_back((double)t.left_curr_x, (double)t.left_curr_y);
        uvs_r.emplace_back(t.has_right_curr ? (double)t.right_curr_x : nan_d,
                           t.has_right_curr ? (double)t.right_curr_y : nan_d);
        wps.push_back(landmark_world_[t.landmark_id]);
      }
      const double ts_now =
          timestamp;
      const auto ba_t0 = std::chrono::steady_clock::now();

      // ---- Sprint 4 IMU integration block ---------------------------------
      // - The first KF is inserted visual-only (no IMU).
      // - From the second KF on, collect the RAW IMU samples in (last_kf_ts,
      //   ts_now] and hand them to the Tier-2 backend, which owns preintegration
      //   (slamko::LocalSmoother contract). Empty ⇒ visual-only insert.
      // - Bootstrap velocity from the visual pose delta between consecutive
      //   KFs. BA then refines velocity + bias.
      std::vector<slamko::ImuSample> imu_for_kf;
      if (enable_imu_ && have_last_kf_) {
        if (!T_BS_resolved_) {
          // The node resolves cam→imu from TF and hands it to us via
          // setExtrinsics(); the pipeline stays ROS/TF-free.
          if (have_provided_T_BS_) {
            T_BS_           = provided_T_BS_;
            ba_cfg_.T_BS    = provided_T_BS_;
            T_BS_resolved_  = true;
            // Push the extrinsic into the Tier-2 backend; it rebuilds its window
            // so the new T_BS takes effect cleanly (LocalBA-rebuild parity).
            smoother_->setExtrinsics(slamko::SE3(provided_T_BS_));
            have_last_kf_ = false;     // restart KF chain to fresh-init VI
            VIO_LOG(
                "resolved T_BS (cam→imu): R [%.3f %.3f %.3f] t [%.3f %.3f %.3f]",
                T_BS_(0,0), T_BS_(1,1), T_BS_(2,2),
                T_BS_(0,3), T_BS_(1,3), T_BS_(2,3));
          }
        }
        if (T_BS_resolved_ && !gravity_calibrated_) {
          // One-shot gravity calibration on the first VI window: accel mean
          // in body ≈ -g_body. Transform to the (visual) world via T_BS and
          // current T_w_c (which is the visual estimate at this KF).
          std::vector<slamko_vio::ImuSample> warmup;
          {
            std::lock_guard<std::mutex> lk(imu_mutex_);
            for (const auto& s : imu_buffer_) {
              if ((int)warmup.size() >= imu_init_warmup_samples_) break;
              warmup.push_back(s);
            }
          }
          if ((int)warmup.size() >= imu_init_warmup_samples_) {
            Eigen::Vector3d a_sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d w_sum = Eigen::Vector3d::Zero();
            for (const auto& s : warmup) { a_sum += s.a; w_sum += s.w; }
            const Eigen::Vector3d a_mean = a_sum / (double)warmup.size();
            const Eigen::Vector3d w_mean = w_sum / (double)warmup.size();
            // R_body_to_world at this KF: T_w_b = T_BS * T_w_c, so
            // R_b_to_w = (T_BS_R * R_wc)^T.
            const Eigen::Matrix3d R_w_b = T_BS_.block<3, 3>(0, 0)
                                        * T_w_c_.block<3, 3>(0, 0);
            const Eigen::Matrix3d R_b_w = R_w_b.transpose();
            // OKVIS-style init: take gravity DIRECTION from the accel mean but
            // LOCK its MAGNITUDE to 9.81. The raw warmup mean has |a|!=9.81 when
            // the body accelerates during init (MH_01 starts in motion →
            // |raw|≈10.97, a 12% error that made the IMU factors fight the
            // stereo-scaled visual estimate). Stereo gives metric scale, so we
            // never estimate g magnitude — only its direction. See OKVIS2-X
            // ImuError.cpp:828 (g_W = imuParams.g * up).
            const Eigen::Vector3d g_raw = R_b_w * (-a_mean);
            ba_cfg_.gravity_w = kGravityMag * g_raw.normalized();
            // Gyro bias: estimated PROPERLY below from the visual rotation
            // chain (handled per-frame after PnP). The warmup-mean approach
            // failed on MH_01 (body in motion at start). Accel bias starts 0.
            bias_lin_.ba = Eigen::Vector3d::Zero();
            (void)w_mean;
            VIO_LOG(
                "calibrated gravity_w = [%.3f %.3f %.3f] |g|=%.2f "
                "(raw |a_mean|=%.2f, locked to %.2f), "
                "bias_g_seed = [%.4f %.4f %.4f] rad/s",
                ba_cfg_.gravity_w.x(), ba_cfg_.gravity_w.y(),
                ba_cfg_.gravity_w.z(), ba_cfg_.gravity_w.norm(),
                a_mean.norm(), kGravityMag,
                bias_lin_.bg.x(), bias_lin_.bg.y(), bias_lin_.bg.z());
            gravity_calibrated_ = true;
            // Push calibrated gravity (+ the IMU noise/bias-RW the backend
            // preintegrates with) into the Tier-2 backend so the first IMU
            // factor sees the right physics. Rebuilds the window — clears prior
            // visual-only KFs (LocalBA-rebuild parity). T_BS is preserved.
            slamko::ImuParams params;
            params.gravity             = ba_cfg_.gravity_w;
            params.accel_noise_density = imu_noise_.accel_noise_density;
            params.gyro_noise_density  = imu_noise_.gyro_noise_density;
            // bias random-walk: the only consumer is LocalBA's between-KF bias
            // factor (ba_cfg_.bias_rw_*); ImuPreintegration ignores it. Pass the
            // LocalBA values so the IMU factor stays bit-for-bit baseline.
            params.accel_bias_rw       = ba_cfg_.bias_rw_accel;
            params.gyro_bias_rw        = ba_cfg_.bias_rw_gyro;
            smoother_->setImuParams(params);
            have_last_kf_      = false;
            imu_initialised_   = false;
            // Re-insert current frame as the new oldest KF below, after the
            // visual-only path. Skip the IMU branch this cycle.
          }
        }
        // ---- Gyro-bias init from visual rotation chain --------------------
        // For each consecutive KF pair, compare the visual body rotation
        // (R_wb_curr * R_wb_prev^T) over dt to the mean IMU gyro reading
        // over the same interval. bias_g = mean(ω_imu - ω_visual). After
        // N=15 samples lock the linearisation bias and let the IMU factor
        // start contributing.
        if (T_BS_resolved_ && gravity_calibrated_ && !bias_g_initialised_) {
          const Eigen::Matrix3d R_BS_R = T_BS_.block<3, 3>(0, 0);
          const Eigen::Matrix3d R_w_b = R_BS_R * T_w_c_.block<3, 3>(0, 0);
          if (has_R_w_b_prev_) {
            const double dt = ts_now - bias_init_prev_ts_;
            if (dt > 1.0e-3) {
              // Body-frame angular velocity from visual: in Forster's notation
              //   ω_B · dt = Log(R_W_B_i^T · R_W_B_{i+1})
              // We store R_w_b ≡ R_W_to_B = R_W_B^T (world-to-body), so the
              // body-frame angular rate is Log(R_w_b_prev · R_w_b_curr^T) / dt.
              const Eigen::Matrix3d dR = R_w_b_prev_ * R_w_b.transpose();
              const Eigen::AngleAxisd aa((Eigen::Matrix3d(dR)));
              const Eigen::Vector3d omega_visual =
                  aa.axis() * (aa.angle() / dt);
              // Mean IMU gyro over (prev_ts, ts_now].
              auto samples = drain_imu_window(bias_init_prev_ts_, ts_now);
              if ((int)samples.size() >= 2) {
                Eigen::Vector3d w_sum = Eigen::Vector3d::Zero();
                for (const auto& s : samples) w_sum += s.w;
                const Eigen::Vector3d omega_imu =
                    w_sum / (double)samples.size();
                bias_g_estimate_ += (omega_imu - omega_visual);
                ++bias_g_count_;
                if (bias_g_count_ >= bias_init_samples_) {
                  bias_lin_.bg = bias_g_estimate_ / (double)bias_g_count_;
                  bias_g_initialised_ = true;
                  VIO_LOG(
                      "gyro bias init from visual chain: [%.5f %.5f %.5f] rad/s "
                      "(from %d KF pairs)",
                      bias_lin_.bg.x(), bias_lin_.bg.y(), bias_lin_.bg.z(),
                      bias_g_count_);
                }
              }
            }
          }
          R_w_b_prev_       = R_w_b;
          has_R_w_b_prev_   = true;
          bias_init_prev_ts_ = ts_now;
        }

        if (T_BS_resolved_ && gravity_calibrated_ && bias_g_initialised_) {
          // Pull IMU samples spanning the inter-KF interval.
          auto samples = drain_imu_window(last_kf_ts_, ts_now);
          if ((int)samples.size() >= 2) {
            // Sandwich the interval: insert bracketing virtual samples at
            // the exact endpoints by clamping timestamps. Forster integrate
            // uses (sample[i-1].w, sample[i-1].a) over (t[i] - t[i-1]).
            samples.front().t = std::max(samples.front().t, last_kf_ts_);
            samples.back().t  = std::min(samples.back().t,  ts_now);

            // Bootstrap velocity from visual delta if not yet initialised.
            if (!imu_initialised_) {
              const Eigen::Vector3d p_prev_w = T_w_c_at_last_kf_.inverse()
                                                  .block<3, 1>(0, 3);
              const Eigen::Vector3d p_curr_w = T_w_c_.inverse()
                                                  .block<3, 1>(0, 3);
              const double dt_v = std::max(1.0e-3, ts_now - last_kf_ts_);
              velocity_w_ = (p_curr_w - p_prev_w) / dt_v;
              imu_initialised_ = true;
            }

            // Hand the backend the RAW (clamped) samples — it owns
            // preintegration. CeresLocalSmoother runs the identical klt_vo
            // ImuPreintegration internally, so this is the baseline numerically.
            imu_for_kf.reserve(samples.size());
            for (const auto& s : samples) {
              slamko::ImuSample x;
              x.timestamp = s.t;
              x.accel     = s.a;
              x.gyro      = s.w;
              imu_for_kf.push_back(x);
            }
          }
        }
      }

      // Cross into the Tier-2 contract. Parallel arrays → StereoObservation IN
      // ORDER (world_init seeds new landmarks positionally — a reorder would
      // rebind seeds). Pose T_w_c_ (world→cam) → T_WB (body→world):
      // T_WB = (E·T_w_c)⁻¹ with E = T_BS_ (cam→body). Empty imu_for_kf ⇒ the
      // backend does a visual-only insert (exact used_imu_insert parity).
      std::vector<slamko::StereoObservation> observations;
      observations.reserve(lids.size());
      for (std::size_t i = 0; i < lids.size(); ++i) {
        slamko::StereoObservation o;
        o.landmark_id = lids[i];
        o.uv_left     = uvs_l[i];
        o.uv_right    = uvs_r[i];     // NaN-x already marks "no stereo match"
        o.world_init  = wps[i];
        observations.push_back(o);
      }
      const slamko::SE3 T_WB_init(Eigen::Matrix4d((T_BS_ * T_w_c_).inverse()));
      slamko::ImuBias bias_core;
      bias_core.gyro  = bias_lin_.bg;
      bias_core.accel = bias_lin_.ba;
      smoother_->insertKeyframe(ts_now, T_WB_init, velocity_w_, bias_core,
                                imu_for_kf, observations);
      // R2 maturity counter: each KF observation matures the landmark.
      for (std::uint32_t lid : lids) ++landmark_obs_count_[lid];
      int n_live_landmarks = 0;
      if (smoother_->optimize()) {
        ba_solved = true;
        // latestPose() is T_WB; back to T_w_c_: T_w_c = E⁻¹·T_WB⁻¹ (E = T_BS_).
        // optimize()==true ⇒ a latest KF exists, so this is always valid (the
        // baseline's latest_pose guard only failed on an empty window).
        T_w_c_ = T_BS_.inverse() * smoother_->latestPose().matrix().inverse();
        world_pose_ = T_w_c_.inverse().cast<float>();
        // Pull back refined landmark world positions for IDs still in window.
        for (std::uint32_t lid : lids) {
          Eigen::Vector3d p_refined;
          if (smoother_->landmark(lid, p_refined)) {
            landmark_world_[lid] = p_refined;
            ++n_live_landmarks;
          }
        }
        // Pull back refined velocity + bias for next-interval preintegration.
        // (LocalBA's getters never fail post-insert ⇒ unconditional, as baseline.)
        if (enable_imu_ && T_BS_resolved_) {
          velocity_w_ = smoother_->latestVelocity();
          const slamko::ImuBias b = smoother_->latestBias();
          bias_lin_.bg = b.gyro;
          bias_lin_.ba = b.accel;
        }
      }
      last_kf_ts_ = ts_now;
      // Record this KF's body pose + 2D landmark observations for the current submap
      // epoch (T_w_c_ is the refined cam pose post-optimize; T_WB = (T_BS·T_w_c)⁻¹).
      // The observations are the BA substrate (each (kf, lid, uv) is one reprojection
      // factor — docs/PLAN_BA_GLOBAL.md) and the input for real two-image LightGlue.
      // Stereo block uses StereoObservation's NaN-x convention per-row when the KF
      // mixes stereo and mono observations (`hasStereo()` then means "uv_right is
      // populated; individual rows may be mono — check isfinite(uv_right(k,0))").
      {
        slamko::KeyframePose kfp;
        kfp.id = static_cast<std::uint64_t>(kf_poses_.size());
        kfp.timestamp = ts_now;
        kfp.T_WB = slamko::SE3(Eigen::Matrix4d((T_BS_ * T_w_c_).inverse()));

        slamko::KeyframeObservations ko;
        const int N = static_cast<int>(observations.size());
        ko.landmark_ids.reserve(N);
        ko.uv.resize(N, 2);
        bool any_stereo = false;
        for (const auto& o : observations) if (o.hasRight()) { any_stereo = true; break; }
        if (any_stereo) ko.uv_right.resize(N, 2);
        for (int k = 0; k < N; ++k) {
          const auto& o = observations[k];
          ko.landmark_ids.push_back(o.landmark_id);
          ko.uv(k, 0) = static_cast<float>(o.uv_left.x());
          ko.uv(k, 1) = static_cast<float>(o.uv_left.y());
          if (any_stereo) {
            if (o.hasRight()) {
              ko.uv_right(k, 0) = static_cast<float>(o.uv_right.x());
              ko.uv_right(k, 1) = static_cast<float>(o.uv_right.y());
            } else {  // NaN-x marks the mono-only row (matches StereoObservation)
              ko.uv_right(k, 0) = std::numeric_limits<float>::quiet_NaN();
              ko.uv_right(k, 1) = 0.f;
            }
          }
        }
        // Capture this KF's VPR descriptor (the current frame's EigenPlaces vector).
        // Per-KF retrieval is the magistrale-return fix: one descriptor per submap
        // aggregates 10 m of trajectory and loses the start-room signal; per-KF lets
        // the relocalizer score each KF independently (max-cosine over a submap's
        // KFs). See docs/PLAN_BA_GLOBAL.md "VPR retrieval: change granularity first".
        if (current_global_desc_.size() > 0) ko.global_descriptor = current_global_desc_;
        kf_poses_.push_back({submap_epoch_, std::move(kfp), std::move(ko)});
      }
      // n_ba_landmarks: live landmarks observed this KF (LocalBA's total-window
      // landmark_count isn't in the LocalSmoother contract). Debug CSV only —
      // not part of any gate.
      n_ba_landmarks = n_live_landmarks;
      first_kf_inliers_     = std::max(first_kf_inliers_, n_pnp_in);
      T_w_c_at_last_kf_     = T_w_c_;
      have_last_kf_         = true;
      frames_since_last_kf_ = 0;
      // Fix A: snapshot the left-cam pixel at this KF; parallax for next-KF
      // decision is computed against this.
      for (auto& t : tracks_) {
        t.left_at_last_kf_x = t.left_curr_x;
        t.left_at_last_kf_y = t.left_curr_y;
        t.has_at_last_kf    = true;
      }
      const auto ba_t1 = std::chrono::steady_clock::now();
      ms_ba = std::chrono::duration<float, std::milli>(ba_t1 - ba_t0).count();
    }

    // Commit this-frame triangulation → previous-frame anchor for next PnP.
    for (auto& t : tracks_) {
      if (t.has_3d_curr) {
        t.point_3d_prev = t.point_3d_curr;
        t.has_3d_prev   = true;
      } else {
        t.has_3d_prev   = false;
      }
    }

    cudaEventRecord(ev_stop_);
    cudaEventSynchronize(ev_stop_);
    float ms_total = 0.f;
    cudaEventElapsedTime(&ms_total, ev_start_, ev_stop_);

    if (csv_out_.is_open()) {
      csv_out_ << frame_idx_ << "," << ms_total << "," << ms_klt << ","
               << ms_stereo << "," << ms_pnp << "," << ms_detect << ","
               << ms_ba << ","
               << n_prev << "," << n_active << "," << n_3d_prev << ","
               << n_pnp_in << "," << n_stereo_match << ","
               << n_new << "," << tracks_.size() << ","
               << n_ba_landmarks << "," << (ba_solved ? 1 : 0)
               << "," << n_lmp_attempt << "," << n_lmp_promote
               << "," << ms_lmp << "\n";
      if ((frame_idx_ % 20) == 0) csv_out_.flush();
    }

    // Health probes for the (future) loop supervisor: loss trigger is the
    // odometry stale-gap (seconds in dead-reckoning), NOT a covariance spike.
    health_.tracking_inlier_ratio =
        (n_3d_prev > 0) ? (float)n_pnp_in / (float)n_3d_prev : -1.f;
    health_.odom_stale_gap_s = in_dead_reckoning_ ? (frame_ts - dr_start_ts_) : 0.0;

    tracker_->swap_pyramids();
    tracker_has_prev_ = true;
    last_frame_ts_ = frame_ts;   // every frame, for dead-reckoning dt
    ++frame_idx_;

    if ((frame_idx_ % 50) == 0) {
      const auto t = world_pose_.block<3,1>(0,3);
      VIO_LOG(
                  "frame=%u  ms_total=%.2f ms_klt=%.2f ms_stereo=%.2f ms_pnp=%.2f  "
                  "active=%d 3d_prev=%d pnp_in=%d stereo=%d  pose=[%.2f,%.2f,%.2f]",
                  frame_idx_, ms_total, ms_klt, ms_stereo, ms_pnp,
                  n_active, n_3d_prev, n_pnp_in, n_stereo_match,
                  t.x(), t.y(), t.z());
    }
}

void VioPipeline::addImu(const ImuSample& s) {
  std::lock_guard<std::mutex> lk(imu_mutex_);
  imu_buffer_.push_back(s);
  // Trim ancient samples (keep ~30 s worth).
  while (!imu_buffer_.empty() &&
         imu_buffer_.back().t - imu_buffer_.front().t > 30.0) {
    imu_buffer_.pop_front();
  }
}

void VioPipeline::setExtrinsics(const Eigen::Matrix4d& T_BS) {
  provided_T_BS_ = T_BS;
  have_provided_T_BS_ = true;
}

}  // namespace slamko_vio
