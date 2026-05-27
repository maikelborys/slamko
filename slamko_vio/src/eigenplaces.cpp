// SPDX-License-Identifier: Apache-2.0
// EigenPlaces TensorRT-10 wrapper — see header. TRT skeleton (build / construct_network
// / save+deserialize engine / BufferManager + enqueueV3) mirrors xfeat.cpp; the only
// EigenPlaces-specific part is process_input (grayscale→3ch + resize + ImageNet-norm,
// CHW) and the single 512-D output. The export bakes a static 1×3×S×S input so no TRT
// optimization profile is needed.

#include "slamko_vio/feature/eigenplaces.h"

#include <fstream>
#include <vector>

using namespace tensorrt_log;
using namespace tensorrt_buffer;

// ImageNet normalization (the EigenPlaces/torchvision training transform; must match
// scripts/export_eigenplaces_onnx.py's reference or retrieval recall collapses silently).
namespace {
constexpr float kMean[3] = {0.485f, 0.456f, 0.406f};
constexpr float kStd[3]  = {0.229f, 0.224f, 0.225f};
}  // namespace

EigenPlaces::EigenPlaces(const EigenPlacesConfig& cfg)
    : cfg_(cfg), engine_(nullptr), stream_(nullptr) {
  setReportableSeverity(Logger::Severity::kINTERNAL_ERROR);
  cudaStreamCreate(&stream_);
}

EigenPlaces::~EigenPlaces() {
  if (stream_) {
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
}

bool EigenPlaces::build() {
  if (deserialize_engine()) return true;
  auto builder = TensorRTUniquePtr<nvinfer1::IBuilder>(
      nvinfer1::createInferBuilder(gLogger.getTRTLogger()));
  if (!builder) return false;
  auto network = TensorRTUniquePtr<nvinfer1::INetworkDefinition>(
      builder->createNetworkV2(0));
  if (!network) return false;
  auto config =
      TensorRTUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) return false;
  auto parser = TensorRTUniquePtr<nvonnxparser::IParser>(
      nvonnxparser::createParser(*network, gLogger.getTRTLogger()));
  if (!parser) return false;

  if (!construct_network(builder, network, config, parser)) return false;

  auto profile_stream = makeCudaStream();
  if (!profile_stream) return false;
  config->setProfileStream(*profile_stream);

  TensorRTUniquePtr<nvinfer1::IHostMemory> plan{
      builder->buildSerializedNetwork(*network, *config)};
  if (!plan) return false;
  TensorRTUniquePtr<nvinfer1::IRuntime> runtime{
      nvinfer1::createInferRuntime(gLogger.getTRTLogger())};
  if (!runtime) return false;
  engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
      runtime->deserializeCudaEngine(plan->data(), plan->size()));
  if (!engine_) return false;
  save_engine();

  ASSERT(network->getNbInputs() == 1);
  input_dims_ = network->getInput(0)->getDimensions();
  ASSERT(input_dims_.nbDims == 4);
  ASSERT(network->getNbOutputs() == 1);
  output_dims_ = network->getOutput(0)->getDimensions();
  return true;
}

bool EigenPlaces::construct_network(
    TensorRTUniquePtr<nvinfer1::IBuilder>& builder,
    TensorRTUniquePtr<nvinfer1::INetworkDefinition>& network,
    TensorRTUniquePtr<nvinfer1::IBuilderConfig>& config,
    TensorRTUniquePtr<nvonnxparser::IParser>& parser) const {
  (void)network;
  if (!parser->parseFromFile(
          cfg_.onnx_file.c_str(),
          static_cast<int>(gLogger.getReportableSeverity()))) {
    return false;
  }
  config->setFlag(nvinfer1::BuilderFlag::kFP16);
  enableDLA(builder.get(), config.get(), cfg_.dla_core);
  return true;
}

void EigenPlaces::save_engine() {
  if (cfg_.engine_file.empty() || engine_ == nullptr) return;
  nvinfer1::IHostMemory* data = engine_->serialize();
  std::ofstream file(cfg_.engine_file, std::ios::binary);
  if (!file) return;
  file.write(reinterpret_cast<const char*>(data->data()), data->size());
}

bool EigenPlaces::deserialize_engine() {
  std::ifstream file(cfg_.engine_file.c_str(), std::ios::binary);
  if (!file.is_open()) return false;
  file.seekg(0, std::ifstream::end);
  size_t size = file.tellg();
  file.seekg(0, std::ifstream::beg);
  std::vector<char> model_stream(size);
  file.read(model_stream.data(), size);
  file.close();

  nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(gLogger);
  if (runtime == nullptr) return false;
  engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
      runtime->deserializeCudaEngine(model_stream.data(), size));
  return engine_ != nullptr;
}

bool EigenPlaces::process_input(const BufferManager& buffers, const cv::Mat& image) {
  const int S = cfg_.input_size;
  cv::Mat gray;
  if (image.channels() == 3) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  else                       gray = image;
  cv::Mat resized;
  cv::resize(gray, resized, cv::Size(S, S), 0, 0, cv::INTER_LINEAR);

  auto* host = static_cast<float*>(
      buffers.getHostBuffer(cfg_.input_tensor_names[0]));
  if (!host) return false;
  // CHW; grayscale replicated to 3 channels, each ImageNet-normalized.
  const int plane = S * S;
  for (int c = 0; c < 3; ++c) {
    for (int y = 0; y < S; ++y) {
      const uchar* row = resized.ptr<uchar>(y);
      float* dst = host + c * plane + y * S;
      for (int x = 0; x < S; ++x)
        dst[x] = (static_cast<float>(row[x]) / 255.0f - kMean[c]) / kStd[c];
    }
  }
  return true;
}

bool EigenPlaces::infer(const cv::Mat& image, Eigen::VectorXf& descriptor) {
  if (!context_) {
    context_ = std::shared_ptr<nvinfer1::IExecutionContext>(
        engine_->createExecutionContext());
    if (!context_) return false;
  }
  const int S = cfg_.input_size;
  if (!context_->setInputShape(cfg_.input_tensor_names[0].c_str(),
                               nvinfer1::Dims4(1, 3, S, S))) {
    return false;
  }
  BufferManager buffers(engine_, context_.get());
  if (!process_input(buffers, image)) return false;
  if (!buffers.setTensorAddresses(context_.get())) return false;
  buffers.copyInputToDeviceAsync(stream_);
  if (!context_->enqueueV3(stream_)) return false;
  buffers.copyOutputToHostAsync(stream_);
  if (cudaStreamSynchronize(stream_) != cudaSuccess) return false;

  const auto* out = static_cast<const float*>(
      buffers.getHostBuffer(cfg_.output_tensor_names[0]));
  if (!out) return false;
  descriptor.resize(cfg_.desc_dim);
  for (int i = 0; i < cfg_.desc_dim; ++i) descriptor(i) = out[i];
  // The network L2-normalizes; renormalize defensively against FP16 drift.
  const float n = descriptor.norm();
  if (n > 1e-6f) descriptor /= n;
  return true;
}
