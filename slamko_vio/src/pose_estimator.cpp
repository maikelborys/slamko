// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys

#include "slamko_vio/pose_estimator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <cmath>
#include <cstddef>

#include "slamko_vio/pnp_cuda.hpp"

namespace slamko_vio {

PoseEstimator::PoseEstimator() = default;
PoseEstimator::PoseEstimator(const Config& cfg) : cfg_(cfg) {
  if (cfg_.use_cuda_ransac) {
    PnPCuda::Config c;
    c.max_ransac_iters          = cfg_.max_ransac_iters;
    c.reprojection_threshold_px = cfg_.reprojection_threshold_px;
    c.min_inliers               = cfg_.min_inliers;
    pnp_cuda_ = std::make_unique<PnPCuda>(cfg_.cuda_max_points, c);
  }
}
PoseEstimator::~PoseEstimator() = default;

namespace {

// Both-camera reprojection cost (cuVSLAM Eq. 4). Residual is 4-d when the
// right-camera observation is finite, 2-d when only the left observation is
// available (we still emit 4 residuals — right pair zeroed — for a fixed
// block size; the LM weight stays consistent).
struct StereoMotionCost {
  StereoMotionCost(double u_l, double v_l, double u_r, double v_r,
                   double fx, double fy, double cx, double cy,
                   double baseline_m, double X, double Y, double Z,
                   bool has_right)
      : u_l_(u_l), v_l_(v_l), u_r_(u_r), v_r_(v_r),
        fx_(fx), fy_(fy), cx_(cx), cy_(cy), b_(baseline_m),
        X_(X), Y_(Y), Z_(Z), has_right_(has_right) {}

  template <typename T>
  bool operator()(const T* const aa, const T* const t, T* res) const {
    const T pw[3] = {T(X_), T(Y_), T(Z_)};
    T pc[3];
    ceres::AngleAxisRotatePoint(aa, pw, pc);
    pc[0] += t[0];
    pc[1] += t[1];
    pc[2] += t[2];
    if (pc[2] < T(1.0e-3)) {
      res[0] = res[1] = res[2] = res[3] = T(0.0);
      return false;
    }
    const T inv_z = T(1.0) / pc[2];
    res[0] = T(fx_) * pc[0] * inv_z + T(cx_) - T(u_l_);
    res[1] = T(fy_) * pc[1] * inv_z + T(cy_) - T(v_l_);
    if (has_right_) {
      res[2] = T(fx_) * (pc[0] - T(b_)) * inv_z + T(cx_) - T(u_r_);
      res[3] = T(fy_) *  pc[1]          * inv_z + T(cy_) - T(v_r_);
    } else {
      res[2] = T(0.0);
      res[3] = T(0.0);
    }
    return true;
  }

  static ceres::CostFunction* Create(double u_l, double v_l,
                                     double u_r, double v_r,
                                     double fx, double fy,
                                     double cx, double cy,
                                     double baseline_m,
                                     double X, double Y, double Z,
                                     bool has_right) {
    return new ceres::AutoDiffCostFunction<StereoMotionCost, 4, 3, 3>(
        new StereoMotionCost(u_l, v_l, u_r, v_r,
                             fx, fy, cx, cy, baseline_m,
                             X, Y, Z, has_right));
  }

 private:
  double u_l_, v_l_, u_r_, v_r_;
  double fx_, fy_, cx_, cy_, b_;
  double X_, Y_, Z_;
  bool   has_right_;
};

// Reproject one landmark through (aa, t) and return the larger of the
// left/right pixel errors (∞ for "behind camera"). Used by the R4 second
// pass to prune residual outliers.
double max_reproj_error(const double aa[3], const double t[3],
                        double X, double Y, double Z,
                        double u_l, double v_l,
                        bool has_right, double u_r, double v_r,
                        const StereoIntrinsics& K) {
  const double pw[3] = {X, Y, Z};
  double pc[3];
  ceres::AngleAxisRotatePoint(aa, pw, pc);
  pc[0] += t[0]; pc[1] += t[1]; pc[2] += t[2];
  if (pc[2] < 1.0e-3) return std::numeric_limits<double>::infinity();
  const double inv_z = 1.0 / pc[2];
  const double ex_l = K.fx * pc[0] * inv_z + K.cx - u_l;
  const double ey_l = K.fy * pc[1] * inv_z + K.cy - v_l;
  double err = std::hypot(ex_l, ey_l);
  if (has_right) {
    const double ex_r = K.fx * (pc[0] - K.baseline_m) * inv_z + K.cx - u_r;
    const double ey_r = K.fy *  pc[1]                  * inv_z + K.cy - v_r;
    err = std::max(err, std::hypot(ex_r, ey_r));
  }
  return err;
}

// Refine (aa, t) on a set of indices into the inlier arrays. Returns the
// number of residual blocks added; if too few, the refine is skipped.
int ceres_refine(double aa[3], double t[3],
                 const std::vector<int>& subset,
                 const std::vector<cv::Point3f>& objp,
                 const std::vector<cv::Point2f>& imgp_l,
                 const std::vector<cv::Point2f>& imgp_r,
                 const std::vector<unsigned char>& has_r,
                 const StereoIntrinsics& K,
                 int max_iters, double function_tol) {
  if (max_iters <= 0) return (int)subset.size();  // skip refine
  ceres::Problem problem;
  problem.AddParameterBlock(aa, 3);
  problem.AddParameterBlock(t, 3);
  int added = 0;
  auto* loss = new ceres::CauchyLoss(3.0);
  for (int idx : subset) {
    auto* cost = StereoMotionCost::Create(
        imgp_l[idx].x, imgp_l[idx].y,
        imgp_r[idx].x, imgp_r[idx].y,
        K.fx, K.fy, K.cx, K.cy, K.baseline_m,
        objp[idx].x, objp[idx].y, objp[idx].z,
        (bool)has_r[idx]);
    problem.AddResidualBlock(cost, loss, aa, t);
    ++added;
  }
  if (added < 6) return added;
  ceres::Solver::Options opts;
  opts.linear_solver_type = ceres::DENSE_QR;
  opts.max_num_iterations = max_iters;
  opts.function_tolerance = function_tol;
  opts.minimizer_progress_to_stdout = false;
  opts.num_threads = 1;
  ceres::Solver::Summary summary;
  ceres::Solve(opts, &problem, &summary);
  return added;
}

}  // namespace

bool PoseEstimator::solve(
    const std::vector<Eigen::Vector3f>& landmarks_3d,
    const std::vector<Eigen::Vector2f>& observations_2d_left,
    const std::vector<Eigen::Vector2f>& observations_2d_right,
    const StereoIntrinsics& K,
    Eigen::Matrix4f& T_prev_to_cur,
    std::vector<int>& inlier_indices,
    bool use_guess) const {
  const std::size_t n = landmarks_3d.size();
  if (n != observations_2d_left.size() || (int)n < cfg_.min_inliers) {
    return false;
  }
  const bool right_provided = (observations_2d_right.size() == n);

  // Determinism: seed OpenCV's RNG once per call. Without this, run-to-run
  // ATE varies by ~3× because PnP RANSAC takes different inlier subsets and
  // the Ceres motion-only BA can converge to slightly different minima. The
  // seed depends on the input size and the first landmark's pixels so it
  // varies between frames but is reproducible across runs.
  if (n > 0) {
    const float kx = observations_2d_left[0].x();
    const float ky = observations_2d_left[0].y();
    const std::uint64_t s = (std::uint64_t)n
                          ^ ((std::uint64_t)(kx * 1000.0f) << 13)
                          ^ ((std::uint64_t)(ky * 1000.0f) << 27);
    cv::theRNG().state = (s == 0) ? 1ULL : s;
  }

  std::vector<cv::Point3f>     objp(n);
  std::vector<cv::Point2f>     imgp_l(n);
  std::vector<cv::Point2f>     imgp_r(n);
  std::vector<unsigned char>   has_r(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    objp[i]   = cv::Point3f(landmarks_3d[i].x(), landmarks_3d[i].y(),
                             landmarks_3d[i].z());
    imgp_l[i] = cv::Point2f(observations_2d_left[i].x(),
                             observations_2d_left[i].y());
    if (right_provided) {
      const Eigen::Vector2f& r = observations_2d_right[i];
      const bool finite = std::isfinite(r.x()) && std::isfinite(r.y()) &&
                          r.x() > 0.f;
      imgp_r[i] = finite ? cv::Point2f(r.x(), r.y()) : cv::Point2f(0.f, 0.f);
      has_r[i]  = finite ? 1 : 0;
    } else {
      imgp_r[i] = cv::Point2f(0.f, 0.f);
      has_r[i]  = 0;
    }
  }

  const cv::Matx33d K_cv(K.fx, 0.0, K.cx,
                          0.0, K.fy, K.cy,
                          0.0, 0.0, 1.0);
  const cv::Mat dist;  // rectified

  cv::Mat rvec, tvec;
  if (use_guess) {
    Eigen::Matrix3d R_guess = T_prev_to_cur.block<3, 3>(0, 0).cast<double>();
    Eigen::Vector3d t_guess = T_prev_to_cur.block<3, 1>(0, 3).cast<double>();
    cv::Matx33d R_cv(R_guess(0,0), R_guess(0,1), R_guess(0,2),
                     R_guess(1,0), R_guess(1,1), R_guess(1,2),
                     R_guess(2,0), R_guess(2,1), R_guess(2,2));
    cv::Rodrigues(R_cv, rvec);
    tvec = (cv::Mat_<double>(3, 1) << t_guess.x(), t_guess.y(), t_guess.z());
  }
  double aa[3] = {0.0, 0.0, 0.0};
  double t_[3] = {0.0, 0.0, 0.0};
  bool ok = false;

  if (cfg_.use_cuda_ransac && pnp_cuda_) {
    Eigen::Matrix4f T_seed = Eigen::Matrix4f::Identity();
    std::vector<int> cuda_inliers;
    const std::uint64_t cuda_seed = (cv::theRNG().state == 0) ? 1ULL
                                                              : cv::theRNG().state;
    // Phase A only: GPU RANSAC, Ceres refine on CPU. Phase B (motion_ba LM
    // either as standalone or as Ceres pre-conditioner) regressed 5/6 EuRoC
    // sequences in benchmarks — kept as a learning artifact in
    // motion_ba_solver.hpp + test_motion_ba but NOT wired into production.
    ok = pnp_cuda_->solve(landmarks_3d, observations_2d_left, K, T_seed,
                          cuda_inliers, cuda_seed);
    if (!ok || (int)cuda_inliers.size() < cfg_.min_inliers) return false;
    inlier_indices = std::move(cuda_inliers);
    Eigen::Matrix3d R_d = T_seed.block<3, 3>(0, 0).cast<double>();
    double R_arr[9] = {
        R_d(0, 0), R_d(1, 0), R_d(2, 0),
        R_d(0, 1), R_d(1, 1), R_d(2, 1),
        R_d(0, 2), R_d(1, 2), R_d(2, 2)};
    ceres::RotationMatrixToAngleAxis(R_arr, aa);
    t_[0] = T_seed(0, 3); t_[1] = T_seed(1, 3); t_[2] = T_seed(2, 3);
  } else {
    cv::Mat inliers;
    const int ransac_method = use_guess ? cv::SOLVEPNP_ITERATIVE
                                         : cv::SOLVEPNP_EPNP;
    ok = cv::solvePnPRansac(
        objp, imgp_l, K_cv, dist, rvec, tvec,
        use_guess,
        cfg_.max_ransac_iters,
        cfg_.reprojection_threshold_px,
        cfg_.ransac_confidence,
        inliers,
        ransac_method);
    if (!ok || inliers.empty() || (int)inliers.rows < cfg_.min_inliers) {
      return false;
    }
    inlier_indices.clear();
    inlier_indices.reserve(inliers.rows);
    for (int i = 0; i < inliers.rows; ++i) {
      inlier_indices.push_back(inliers.at<int>(i, 0));
    }
    aa[0] = rvec.at<double>(0); aa[1] = rvec.at<double>(1); aa[2] = rvec.at<double>(2);
    t_[0] = tvec.at<double>(0); t_[1] = tvec.at<double>(1); t_[2] = tvec.at<double>(2);
  }

  if (cfg_.refine_lm && cfg_.lm_max_iters > 0) {
    ceres_refine(aa, t_, inlier_indices, objp, imgp_l, imgp_r, has_r, K,
                 cfg_.lm_max_iters, cfg_.lm_function_tol);
    if (cfg_.refine_second_pass && cfg_.refine_pixel_threshold > 0.0f) {
      std::vector<int> tight;
      tight.reserve(inlier_indices.size());
      for (int idx : inlier_indices) {
        const double e = max_reproj_error(
            aa, t_, objp[idx].x, objp[idx].y, objp[idx].z,
            imgp_l[idx].x, imgp_l[idx].y,
            (bool)has_r[idx], imgp_r[idx].x, imgp_r[idx].y, K);
        if (e <= cfg_.refine_pixel_threshold) tight.push_back(idx);
      }
      if ((int)tight.size() >= cfg_.min_inliers) {
        ceres_refine(aa, t_, tight, objp, imgp_l, imgp_r, has_r, K,
                     cfg_.lm_max_iters, cfg_.lm_function_tol);
        inlier_indices.swap(tight);
      }
    }
  }

  // angle_axis → rotation matrix
  double R_arr[9];
  ceres::AngleAxisToRotationMatrix(aa, R_arr);
  Eigen::Matrix3d R_d;
  // ceres::AngleAxisToRotationMatrix uses column-major output by default
  // (R = [r00 r10 r20  r01 r11 r21  r02 r12 r22]).
  R_d << R_arr[0], R_arr[3], R_arr[6],
         R_arr[1], R_arr[4], R_arr[7],
         R_arr[2], R_arr[5], R_arr[8];

  T_prev_to_cur.setIdentity();
  T_prev_to_cur.block<3, 3>(0, 0) = R_d.cast<float>();
  T_prev_to_cur(0, 3) = (float)t_[0];
  T_prev_to_cur(1, 3) = (float)t_[1];
  T_prev_to_cur(2, 3) = (float)t_[2];
  return true;
}

}  // namespace slamko_vio
