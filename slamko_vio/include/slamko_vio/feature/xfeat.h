// SPDX-License-Identifier: Apache-2.0
// XFeat (Verlab accelerated_features, Apache-2.0) TensorRT-10 wrapper.
// Ported from AirSLAM_XFEAT/src/xfeat.cpp (Apache-headed); the algorithm is
// transcribed from accelerated_features/modules/xfeat.py. slamko uses it behind
// slamko_core::FeatureSource (see xfeat_source.hpp). Host post-processing only
// (the optional CUDA post-proc path was dropped to keep the port lean).
//
// Engine I/O (static-shape; export H=480, W=752 by default):
//   input  "image"  (1,1,H,W) float32 in [0,1]
//   output "feats"  (1,64,H/8,W/8) dense descriptors
//   output "keypts" (1,65,H/8,W/8) logits (softmax along axis 1)
//   output "rel"    (1,1,H/8,W/8)  reliability heatmap
// Output features (Eigen 67×N): row0 score, row1 x, row2 y, rows3..66 = 64-d L2 desc.

#ifndef SLAMKO_VIO_XFEAT_H_
#define SLAMKO_VIO_XFEAT_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <memory>
#include <opencv2/opencv.hpp>

#include "buffers.h"

using tensorrt_buffer::TensorRTUniquePtr;

// Plain config (no yaml) — defaults match the EuRoC-native 752×480 export.
struct XFeatConfig {
  int max_keypoints        = 1500;
  float keypoint_threshold = 0.05f;
  int remove_borders       = 4;
  int nms_kernel_size      = 5;
  int input_height         = 480;
  int input_width          = 752;
  int dla_core             = -1;
  std::vector<std::string> input_tensor_names  = {"image"};
  std::vector<std::string> output_tensor_names = {"feats", "keypts", "rel"};
  std::string onnx_file;
  std::string engine_file;
};

// 3 leading rows (score, x, y) + 64 descriptor rows.
inline constexpr int kXFeatFeatureRows = 67;

class XFeat {
 public:
  explicit XFeat(const XFeatConfig& xfeat_config);
  ~XFeat();

  bool build();
  bool infer(const cv::Mat& image,
             Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic>& features);
  void save_engine();
  bool deserialize_engine();

 private:
  int input_width_;
  int input_height_;
  int resized_width_;
  int resized_height_;
  float w_scale_;
  float h_scale_;

  XFeatConfig xfeat_config_;
  nvinfer1::Dims input_dims_{};
  nvinfer1::Dims feats_dims_{};
  nvinfer1::Dims keypts_dims_{};
  nvinfer1::Dims rel_dims_{};
  std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IExecutionContext> context_;
  cudaStream_t stream_;

  bool construct_network(
      TensorRTUniquePtr<nvinfer1::IBuilder>& builder,
      TensorRTUniquePtr<nvinfer1::INetworkDefinition>& network,
      TensorRTUniquePtr<nvinfer1::IBuilderConfig>& config,
      TensorRTUniquePtr<nvonnxparser::IParser>& parser) const;
  bool process_input(const tensorrt_buffer::BufferManager& buffers,
                     const cv::Mat& image);
  bool process_output(
      const tensorrt_buffer::BufferManager& buffers,
      Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic>& features);
};

typedef std::shared_ptr<XFeat> XFeatPtr;

#endif  // SLAMKO_VIO_XFEAT_H_
