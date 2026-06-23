cmake_minimum_required(VERSION 3.22)
project(slamko_ros LANGUAGES CXX)

# slamko_ros — the composition root (Hard Rule #2 exemption: the ONLY package
# allowed to know all the others). P-A ships provider_fusion_node (the loose
# fuser over an external odometry provider) + provider_chain_offline (the
# no-ROS reproducible gate on recorded provider trajectories).

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

# Live Rerun visualizer (rerun.io). OFF by default so a normal build needs no
# network / no extra SDK; ON fetches the Rerun C++ SDK (~one zip from GitHub
# releases) and compiles the real VizSink. The node ALWAYS builds either way —
# viz_sink.cpp is a complete no-op without this. Enable with:
#   colcon build --packages-select slamko_ros --cmake-args -DSLAMKO_WITH_RERUN=ON
option(SLAMKO_WITH_RERUN "Build the live Rerun visualizer sink" OFF)

find_package(ament_cmake REQUIRED)
find_package(slamko_core REQUIRED)
find_package(slamko_loop REQUIRED)
find_package(slamko_vio REQUIRED)   # TRT feature wrappers (EigenPlaces) — composition-root use
find_package(slamko_tsdf REQUIRED)  # live volumetric engine (nvblox hidden behind its PIMPL)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

# TensorRT 10 ships no CMake config (same discovery as slamko_vio).
find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS /usr/include/x86_64-linux-gnu /usr/include)
find_library(NVINFER nvinfer HINTS /usr/lib/x86_64-linux-gnu)
find_library(NVONNXPARSER nvonnxparser HINTS /usr/lib/x86_64-linux-gnu)

# Rerun C++ SDK (only when the viz is enabled).
if(SLAMKO_WITH_RERUN)
  include(FetchContent)
  FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/download/0.33.0/rerun_cpp_sdk.zip)
  FetchContent_MakeAvailable(rerun_sdk)
endif()

add_executable(provider_fusion_node nodes/provider_fusion_node.cpp src/viz_sink.cpp)
target_include_directories(provider_fusion_node PRIVATE
  ${TENSORRT_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(provider_fusion_node
  slamko_core slamko_loop slamko_vio slamko_tsdf rclcpp nav_msgs sensor_msgs
  geometry_msgs tf2_ros ament_index_cpp)
# After ament deps so cudart/TRT land at the END of the link line (slamko_vio_core
# has undefined cudart/nvinfer refs the consumer must satisfy).
target_link_libraries(provider_fusion_node Eigen3::Eigen ${OpenCV_LIBS}
  ${NVINFER} ${NVONNXPARSER} CUDA::cudart)
if(SLAMKO_WITH_RERUN)
  target_compile_definitions(provider_fusion_node PRIVATE SLAMKO_WITH_RERUN)
  target_link_libraries(provider_fusion_node rerun_sdk)

  # No-ROS runtime smoke test of the viz path -> a .rrd capture you can scrub.
  add_executable(viz_selftest tools/viz_selftest.cpp src/viz_sink.cpp)
  target_include_directories(viz_selftest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_definitions(viz_selftest PRIVATE SLAMKO_WITH_RERUN)
  ament_target_dependencies(viz_selftest slamko_core)
  target_link_libraries(viz_selftest Eigen3::Eigen ${OpenCV_LIBS} rerun_sdk)
  install(TARGETS viz_selftest RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()
install(TARGETS provider_fusion_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

add_executable(provider_chain_offline tools/provider_chain_offline.cpp)
target_link_libraries(provider_chain_offline Eigen3::Eigen)
ament_target_dependencies(provider_chain_offline slamko_core slamko_loop)
install(TARGETS provider_chain_offline RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
