// SPDX-License-Identifier: Apache-2.0
// EigenPlaces (gmberton/EigenPlaces, MIT) TensorRT-10 wrapper — a GLOBAL place-
// recognition descriptor for loop-closure RETRIEVAL. XFeat-64 local descriptors carry
// no place-level signal (a genuine loop return is indistinguishable from a place 80 m
// away — proven), so the never-lost relocalizer never recognizes a revisit. EigenPlaces
// produces ONE 512-D L2-normalized vector per keyframe; cosine-NN over those retrieves
// candidate keyframes, which the existing XFeat+PnP stage verifies geometrically
// (that stage already works at 100% on a true match). Validated on TUM VI magistrale1:
// Recall@5 = 1.0 on the start-room return. See docs/PLAN_VPR_RELOC.md.
//
// Engine I/O (static-shape; export via scripts/export_eigenplaces_onnx.py):
//   input  "image"      (1,3,S,S) float32 — grayscale→3ch, resized to SxS, ImageNet-norm
//   output "descriptor" (1,512)   float32 — already L2-normalized
// Mirrors xfeat.cpp's TensorRT contract (build / deserialize / BufferManager / enqueueV3).

#ifndef SLAMKO_VIO_EIGENPLACES_H_
#define SLAMKO_VIO_EIGENPLACES_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>

#include "buffers.h"

using tensorrt_buffer::TensorRTUniquePtr;

struct EigenPlacesConfig {
  int input_size = 512;   // square network input (training eval size)
  int desc_dim   = 512;   // output descriptor dimension
  int dla_core   = -1;
  std::vector<std::string> input_tensor_names  = {"image"};
  std::vector<std::string> output_tensor_names = {"descriptor"};
  std::string onnx_file;
  std::string engine_file;
};

class EigenPlaces {
 public:
  explicit EigenPlaces(const EigenPlacesConfig& cfg);
  ~EigenPlaces();

  bool build();
  // Compute the global descriptor of a (grayscale or color) image. Returns a
  // desc_dim L2-normalized vector. False on inference failure.
  bool infer(const cv::Mat& image, Eigen::VectorXf& descriptor);
  void save_engine();
  bool deserialize_engine();

 private:
  EigenPlacesConfig cfg_;
  nvinfer1::Dims input_dims_{};
  nvinfer1::Dims output_dims_{};
  std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IExecutionContext> context_;
  cudaStream_t stream_;

  bool construct_network(
      TensorRTUniquePtr<nvinfer1::IBuilder>& builder,
      TensorRTUniquePtr<nvinfer1::INetworkDefinition>& network,
      TensorRTUniquePtr<nvinfer1::IBuilderConfig>& config,
      TensorRTUniquePtr<nvonnxparser::IParser>& parser) const;
  bool process_input(const tensorrt_buffer::BufferManager& buffers, const cv::Mat& image);
};

typedef std::shared_ptr<EigenPlaces> EigenPlacesPtr;

#endif  // SLAMKO_VIO_EIGENPLACES_H_
