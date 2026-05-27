// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Contract wiring: dummy implementations of every abstract interface prove the
// headers compile, link, and the polymorphism works end to end — a frontend
// creating a node + emitting a factor onto a backend, and a feature source
// returning Features. Also checks RobustKernel weights. If this builds and
// passes, the spine is usable by vio/fusion/loop.

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "slamko_core/factor.hpp"
#include "slamko_core/factor_graph_backend.hpp"
#include "slamko_core/feature_source.hpp"
#include "slamko_core/robust_kernel.hpp"
#include "slamko_core/sensor_frontend.hpp"

using namespace slamko;

namespace {

// -------- a minimal in-memory backend over 3-vector nodes -----------------
class DummyBackend : public FactorGraphBackend {
 public:
  NodeKey addNode(NodeType type, const Eigen::VectorXd& init, bool = false) override {
    NodeKey k(type, next_id_++);
    values_[k] = init;
    return k;
  }
  void addFactor(const FactorPtr& f) override { factors_.push_back(f); }
  bool optimize() override { return true; }
  Eigen::VectorXd value(NodeKey k) const override { return values_.at(k); }
  Eigen::MatrixXd marginalCovariance(NodeKey k) const override {
    return Eigen::MatrixXd::Identity(values_.at(k).size(), values_.at(k).size());
  }
  void marginalizeOlderThan(double) override {}

  const NodeValues& values() const { return values_; }
  std::size_t numFactors() const { return factors_.size(); }

 private:
  NodeValues values_;
  std::vector<FactorPtr> factors_;
  std::uint64_t next_id_ = 0;
};

// -------- a prior factor on a 3-vector node: r = v - target ----------------
class PriorFactor : public Factor {
 public:
  PriorFactor(NodeKey k, Eigen::Vector3d target) : key_(k), target_(std::move(target)) {}
  std::vector<NodeKey> keys() const override { return {key_}; }
  int dim() const override { return 3; }
  bool evaluate(const NodeValues& v, Eigen::VectorXd& r,
                std::vector<Eigen::MatrixXd>* J) const override {
    r = v.at(key_) - target_;
    if (J) *J = {Eigen::MatrixXd::Identity(3, 3)};
    return true;
  }
  Eigen::MatrixXd sqrt_information() const override {
    return Eigen::MatrixXd::Identity(3, 3);
  }
  RobustKernel robust_kernel() const override { return {RobustKernel::Type::Huber, 1.0}; }

 private:
  NodeKey key_;
  Eigen::Vector3d target_;
};

// -------- a frontend that adds one node + one prior factor -----------------
class DummyFrontend : public SensorFrontend {
 public:
  std::string name() const override { return "dummy"; }
  void process(const Measurement&, const KeyframeTimeline&,
               FactorGraphBackend& backend) override {
    NodeKey k = backend.addNode(NodeType::Velocity, Eigen::Vector3d(1, 2, 3));
    backend.addFactor(std::make_shared<PriorFactor>(k, Eigen::Vector3d(1, 2, 3)));
    last_key_ = k;
  }
  NodeKey last_key() const { return last_key_; }

 private:
  NodeKey last_key_;
};

class EmptyTimeline : public KeyframeTimeline {
 public:
  NodeKey latestPose() const override { return {}; }
  NodeKey poseAt(double) const override { return {}; }
  bool empty() const override { return true; }
};

// -------- a descriptor-less feature source ---------------------------------
class DummySource : public FeatureSource {
 public:
  std::string name() const override { return "dummy_detector"; }
  Features detect(const ImageView&) override {
    Features f;
    f.keypoints.resize(2, 3);
    f.keypoints << 10.f, 20.f, 0.9f,
                   30.f, 40.f, 0.7f;
    return f;
  }
  int descriptorDim() const override { return 0; }
};

}  // namespace

TEST(Contracts, PriorFactorResidual) {
  DummyBackend backend;
  NodeKey k = backend.addNode(NodeType::Velocity, Eigen::Vector3d(2, 0, -1));
  PriorFactor f(k, Eigen::Vector3d(1, 1, 1));

  Eigen::VectorXd r;
  std::vector<Eigen::MatrixXd> J;
  ASSERT_TRUE(f.evaluate(backend.values(), r, &J));
  EXPECT_EQ(r.size(), 3);
  EXPECT_LT((r - Eigen::Vector3d(1, -1, -2)).norm(), 1e-12);
  ASSERT_EQ(J.size(), 1u);
  EXPECT_LT((J[0] - Eigen::MatrixXd::Identity(3, 3)).norm(), 1e-12);
  EXPECT_EQ(f.keys().size(), 1u);
  EXPECT_EQ(f.keys()[0], k);
}

TEST(Contracts, FrontendDrivesBackendPolymorphically) {
  DummyBackend backend;
  EmptyTimeline timeline;
  std::unique_ptr<SensorFrontend> frontend = std::make_unique<DummyFrontend>();
  FactorGraphBackend& b = backend;

  Measurement m;
  m.timestamp = 1.0;
  frontend->process(m, timeline, b);

  EXPECT_EQ(backend.values().size(), 1u);
  EXPECT_EQ(backend.numFactors(), 1u);
  EXPECT_TRUE(b.optimize());
}

TEST(Contracts, FeatureSourcePolymorphic) {
  std::unique_ptr<FeatureSource> src = std::make_unique<DummySource>();
  Features f = src->detect(ImageView{});
  EXPECT_EQ(f.size(), 2);
  EXPECT_FALSE(f.hasDescriptors());
  EXPECT_EQ(src->descriptorDim(), 0);
  EXPECT_FLOAT_EQ(f.keypoints(1, 0), 30.f);
}

TEST(RobustKernelWeights, BehaveAsExpected) {
  RobustKernel none;
  EXPECT_DOUBLE_EQ(none.weight(5.0), 1.0);

  RobustKernel huber(RobustKernel::Type::Huber, 1.0);
  EXPECT_DOUBLE_EQ(huber.weight(0.5), 1.0);          // inlier
  EXPECT_DOUBLE_EQ(huber.weight(2.0), 0.5);          // δ/e = 1/2

  RobustKernel cauchy(RobustKernel::Type::Cauchy, 1.0);
  EXPECT_LT(cauchy.weight(10.0), cauchy.weight(1.0));  // monotone down
  EXPECT_GT(cauchy.weight(10.0), 0.0);

  RobustKernel tukey(RobustKernel::Type::Tukey, 2.0);
  EXPECT_DOUBLE_EQ(tukey.weight(3.0), 0.0);          // beyond support -> rejected

  RobustKernel dcs(RobustKernel::Type::DCS, 1.0);
  EXPECT_DOUBLE_EQ(dcs.weight(0.5), 1.0);            // inlier (e^2 <= Φ)
  EXPECT_LT(dcs.weight(5.0), 1.0);                   // outlier scaled down
}
