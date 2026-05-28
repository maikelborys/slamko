// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// LightGlueMatcher — see header. Port of AirSLAM_XFEAT/src/lighter_glue.cc
// adapted to the slamko_core Features layout + the slamko::Matcher contract.
// All torch code is behind SLAMKO_HAVE_TORCH; the #else branch is a no-op so
// the default build links without libtorch.

#include "slamko_loop/lightglue_matcher.hpp"

#include <algorithm>
#include <numeric>

namespace slamko {

namespace {
// Select up to n rows of a Features by descending keypoint score (col 2),
// preserving original order when nothing is dropped. Returns original indices.
// [[maybe_unused]]: only referenced under SLAMKO_HAVE_TORCH (no-op build skips it).
[[maybe_unused]] std::vector<int> selectTopK(const Features& f, int n) {
  const int N = f.size();
  std::vector<int> idx(N);
  std::iota(idx.begin(), idx.end(), 0);
  if (N > n) {
    std::partial_sort(idx.begin(), idx.begin() + n, idx.end(),
                      [&](int a, int b) {
                        return f.keypoints(a, 2) > f.keypoints(b, 2);
                      });
    idx.resize(n);
  }
  return idx;
}
}  // namespace

}  // namespace slamko

#ifdef SLAMKO_HAVE_TORCH

#include <iostream>

#include <torch/script.h>
#include <torch/cuda.h>

namespace slamko {

struct LightGlueMatcher::Impl {
  torch::jit::script::Module module;
  torch::Device device{torch::kCPU};
};

LightGlueMatcher::LightGlueMatcher(const LightGlueConfig& cfg)
    : cfg_(cfg), impl_(std::make_unique<Impl>()) {}

LightGlueMatcher::~LightGlueMatcher() = default;

bool LightGlueMatcher::build() {
  try {
    if (cfg_.use_cuda && torch::cuda::is_available()) {
      impl_->device = torch::Device(torch::kCUDA, 0);
    } else {
      impl_->device = torch::Device(torch::kCPU);
      if (cfg_.use_cuda)
        std::cerr << "LightGlueMatcher: CUDA unavailable, using CPU\n";
    }
    impl_->module = torch::jit::load(cfg_.model_path, impl_->device);
    impl_->module.eval();
    ready_ = true;
    std::cout << "LightGlueMatcher loaded " << cfg_.model_path << " on "
              << (impl_->device.is_cuda() ? "CUDA" : "CPU")
              << " (N=" << cfg_.trace_n << ")\n";
    return true;
  } catch (const c10::Error& e) {
    std::cerr << "LightGlueMatcher::build() failed: " << e.what() << "\n";
    ready_ = false;
    return false;
  }
}

std::vector<DescMatch> LightGlueMatcher::match(const Features& query,
                                               const Features& train) {
  std::vector<DescMatch> out;
  if (!ready_) return out;
  if (!query.hasDescriptors() || !train.hasDescriptors()) return out;
  if (query.descriptorDim() != 64 || train.descriptorDim() != 64) return out;

  const int N = cfg_.trace_n;
  const std::vector<int> qsel = selectTopK(query, N);
  const std::vector<int> tsel = selectTopK(train, N);
  const int Mq = static_cast<int>(qsel.size());
  const int Mt = static_cast<int>(tsel.size());
  if (Mq == 0 || Mt == 0) return out;

  // Pack contiguous (N,2) keypoint + (N,64) descriptor buffers; pad the tail
  // with out-of-image sentinels (the model attends to them as low-confidence
  // and the score cut drops any that slip through — same as lighter_glue.cc).
  std::vector<float> kp0(N * 2, -1.0f), kp1(N * 2, -1.0f);
  std::vector<float> d0(N * 64, 0.0f), d1(N * 64, 0.0f);
  auto pack = [](const Features& f, const std::vector<int>& sel,
                 std::vector<float>& kp, std::vector<float>& d) {
    const int M = static_cast<int>(sel.size());
    for (int k = 0; k < M; ++k) {
      const int r = sel[k];
      kp[k * 2 + 0] = f.keypoints(r, 0);
      kp[k * 2 + 1] = f.keypoints(r, 1);
      for (int c = 0; c < 64; ++c) d[k * 64 + c] = f.descriptors(r, c);
    }
  };
  pack(query, qsel, kp0, d0);
  pack(train, tsel, kp1, d1);

  auto opts = torch::TensorOptions().dtype(torch::kFloat32);
  torch::Tensor t_k0 = torch::from_blob(kp0.data(), {1, N, 2}, opts).to(impl_->device);
  torch::Tensor t_k1 = torch::from_blob(kp1.data(), {1, N, 2}, opts).to(impl_->device);
  torch::Tensor t_d0 = torch::from_blob(d0.data(), {1, N, 64}, opts).to(impl_->device);
  torch::Tensor t_d1 = torch::from_blob(d1.data(), {1, N, 64}, opts).to(impl_->device);

  torch::Tensor matches0, scores0;
  try {
    std::vector<torch::jit::IValue> inputs{t_k0, t_d0, t_k1, t_d1};
    torch::NoGradGuard nograd;
    auto res = impl_->module.forward(inputs).toTuple();
    matches0 = res->elements()[0].toTensor().to(torch::kCPU).contiguous();
    scores0  = res->elements()[1].toTensor().to(torch::kCPU).contiguous();
  } catch (const c10::Error& e) {
    std::cerr << "LightGlueMatcher::match forward failed: " << e.what() << "\n";
    return out;
  }

  // matches0[0,k] = index into the (padded) train keypoints, or -1. Keep pairs
  // where k < Mq (skip query pad), j < Mt (skip train pad), score >= thresh.
  // Map (k,j) back to the ORIGINAL Features rows via qsel/tsel.
  const int64_t* m = matches0.data_ptr<int64_t>();
  const float*   s = scores0.data_ptr<float>();
  out.reserve(Mq);
  for (int k = 0; k < Mq; ++k) {
    const int64_t j = m[k];
    if (j < 0 || j >= Mt) continue;
    if (s[k] < cfg_.score_thresh) continue;
    DescMatch dm;
    dm.query_idx = static_cast<std::uint32_t>(qsel[k]);
    dm.train_idx = static_cast<std::uint32_t>(tsel[static_cast<int>(j)]);
    dm.score = s[k];
    out.push_back(dm);
  }
  return out;
}

}  // namespace slamko

#else  // !SLAMKO_HAVE_TORCH — no-op fallback (relocalizer reverts to brute force)

namespace slamko {

struct LightGlueMatcher::Impl {};

LightGlueMatcher::LightGlueMatcher(const LightGlueConfig& cfg) : cfg_(cfg) {}
LightGlueMatcher::~LightGlueMatcher() = default;

bool LightGlueMatcher::build() {
  ready_ = false;
  return false;
}

std::vector<DescMatch> LightGlueMatcher::match(const Features&, const Features&) {
  return {};
}

}  // namespace slamko

#endif  // SLAMKO_HAVE_TORCH
