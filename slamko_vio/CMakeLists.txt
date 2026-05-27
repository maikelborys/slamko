cmake_minimum_required(VERSION 3.22)
project(slamko_vio LANGUAGES CXX CUDA)

# slamko_vio — the fast S1 visual-inertial tracker. B1: faithful port of klt_vo
# (CUDA Shi-Tomasi + KLT + stereo + PnP + Ceres LocalBA + IMU). Self-contained
# for now; the slamko_core FeatureSource/FeatureTracker seam wires in at B1b.

option(SLAMKO_VIO_BUILD_TESTS "Build gtest unit tests" ON)
option(SLAMKO_VIO_BUILD_NODE  "Build the ROS 2 node" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# RTX 4070 Laptop = sm_89 (Ada). Override for portability.
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 89)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(slamko_core REQUIRED)
# Node-only: the composition root injects slamko_fusion's GtsamLocalSmoother for
# backend:=gtsam. slamko_vio_core links ONLY slamko_core (Hard Rule #2). GTSAM is
# fully encapsulated inside the shared libslamko_fusion.so (PIMPL), so the node
# links only that — it never finds or links gtsam itself.
find_package(slamko_fusion REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Ceres REQUIRED)
find_package(ament_index_cpp REQUIRED)

# TensorRT 10 (ships no CMake config) — discover headers + libs for the XFeat path.
find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS /usr/include/x86_64-linux-gnu /usr/include)
find_library(NVINFER nvinfer HINTS /usr/lib/x86_64-linux-gnu)
find_library(NVONNXPARSER nvonnxparser HINTS /usr/lib/x86_64-linux-gnu)

# ---------------------------------------------------- core lib (C++/CUDA, no ROS)
add_library(slamko_vio_core
  src/shitomasi.cu
  src/klt_tracker.cu
  src/stereo_matcher.cu
  src/pose_estimator.cpp
  src/local_ba.cpp
  src/ceres_local_smoother.cpp
  src/imu_preintegration.cpp
  src/pnp_cuda.cu
  src/vio_pipeline.cpp
  src/xfeat.cpp
  3rdparty/tensorrtbuffer/src/logger.cpp
)
target_include_directories(slamko_vio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/tensorrtbuffer/include
  ${TENSORRT_INCLUDE_DIR}
)
target_link_libraries(slamko_vio_core
  Eigen3::Eigen ${OpenCV_LIBS} CUDA::cudart Ceres::ceres ${NVINFER} ${NVONNXPARSER})
ament_target_dependencies(slamko_vio_core slamko_core)
target_compile_features(slamko_vio_core PUBLIC cxx_std_17)
set_target_properties(slamko_vio_core PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_RESOLVE_DEVICE_SYMBOLS ON)
target_compile_options(slamko_vio_core PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda --expt-relaxed-constexpr>)

# ------------------------------------------------------------------- ROS 2 node
if(SLAMKO_VIO_BUILD_NODE)
  add_executable(slamko_vio_node nodes/vio_node.cpp)
  target_link_libraries(slamko_vio_node slamko_vio_core)
  ament_target_dependencies(slamko_vio_node
    slamko_core slamko_fusion ament_index_cpp rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs
    std_msgs tf2 tf2_ros cv_bridge image_transport message_filters)
  # libslamko_fusion.so carries its own DT_RPATH to GTSAM 4.3 (/usr/local); add it
  # here too (+ --disable-new-dtags) so the transitive gtsam load can't fall back
  # to the 4.2 at /usr (DT_RUNPATH isn't transitive).
  set_target_properties(slamko_vio_node PROPERTIES
    BUILD_RPATH /usr/local/lib INSTALL_RPATH /usr/local/lib)
  target_link_options(slamko_vio_node PRIVATE -Wl,--disable-new-dtags)
  install(TARGETS slamko_vio_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
endif()

# --------------------------------------------------------------- install + export
install(TARGETS slamko_vio_core
  EXPORT slamko_vio_coreTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# --------------------------------------------------------------------- tests
if(BUILD_TESTING AND SLAMKO_VIO_BUILD_TESTS)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t test_shitomasi test_klt_tracker test_stereo_matcher
            test_imu_preintegration test_p3p test_motion_ba
            test_ceres_local_smoother)
    ament_add_gtest(${t} tests/${t}.cpp)
    target_link_libraries(${t} slamko_vio_core)
  endforeach()
endif()

ament_export_dependencies(slamko_core rclcpp sensor_msgs nav_msgs visualization_msgs cv_bridge Eigen3 OpenCV)
ament_export_include_directories(include)
ament_export_libraries(slamko_vio_core)
ament_package()
