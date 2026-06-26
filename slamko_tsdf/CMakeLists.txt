cmake_minimum_required(VERSION 3.22)
project(slamko_tsdf LANGUAGES CXX)

# slamko_tsdf — the volumetric layer that BENDS with the pose-graph. Pure
# C++/Eigen over slamko_core contracts (Hard Rule #2). The bend lives in
# VolumetricMapper (re-integrate the per-keyframe depth source at corrected
# poses); the GPU TSDF is an interchangeable backend.
#
# nvblox (CUDA) is OFF by default so the workspace + the gtest suite stay
# CUDA-free. The nvblox_backend.cpp is ALWAYS compiled — without the macro it is a
# working no-op (available()=false). Turn it on with:
#   colcon build --packages-select slamko_tsdf --cmake-args -DSLAMKO_WITH_NVBLOX=ON

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

option(SLAMKO_WITH_NVBLOX "Build the nvblox GPU TSDF→ESDF backend" OFF)

find_package(ament_cmake REQUIRED)
find_package(slamko_core REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(slamko_tsdf SHARED
  src/volumetric_mapper.cpp
  src/live_driver.cpp
  src/nvblox_backend.cpp)
target_include_directories(slamko_tsdf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(slamko_tsdf Eigen3::Eigen)
ament_target_dependencies(slamko_tsdf slamko_core)
target_compile_features(slamko_tsdf PUBLIC cxx_std_17)

if(SLAMKO_WITH_NVBLOX)
  # nvblox is built/installed by the nvblox_ros workspace (libnvblox_lib.so +
  # headers under <prefix>/include). Provide it on CMAKE_PREFIX_PATH, or set
  # nvblox_DIR. CUDA toolkit + glog/gflags are transitive nvblox deps.
  find_package(CUDAToolkit REQUIRED)
  # nvbloxConfig.cmake exposes nvblox::nvblox_lib but does NOT pull its own
  # transitive imported targets — find them first or the link interface breaks.
  find_package(glog REQUIRED)
  find_package(gflags REQUIRED)
  find_package(nvblox REQUIRED)
  target_compile_definitions(slamko_tsdf PRIVATE SLAMKO_WITH_NVBLOX)
  target_link_libraries(slamko_tsdf nvblox::nvblox_lib CUDA::cudart)
  message(STATUS "slamko_tsdf: nvblox GPU backend ENABLED")
  # End-to-end dense-channel self-test (synthetic TSDF → query → ICP recovers a drift).
  # Needs slamko_loop's header-only point-to-SDF ICP (registerToSdf).
  find_package(slamko_loop QUIET)
  add_executable(nvblox_sdf_selftest tools/nvblox_sdf_selftest.cpp)
  target_link_libraries(nvblox_sdf_selftest slamko_tsdf Eigen3::Eigen)
  ament_target_dependencies(nvblox_sdf_selftest slamko_core slamko_loop)
  install(TARGETS nvblox_sdf_selftest RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "slamko_tsdf: nvblox backend OFF (no-op stub) — CUDA-free build")
endif()

# Offline driver: submap archive + per-kf depth → re-integrate at corrected poses
# → Nav2 costmap (PGM/YAML) + mesh (PLY). Real output needs -DSLAMKO_WITH_NVBLOX.
add_executable(slamko_tsdf_export src/export_tool.cpp)
target_link_libraries(slamko_tsdf_export slamko_tsdf Eigen3::Eigen)
ament_target_dependencies(slamko_tsdf_export slamko_core)

install(TARGETS slamko_tsdf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS slamko_tsdf_export RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_volumetric_mapper test/test_volumetric_mapper.cpp)
  target_link_libraries(test_volumetric_mapper slamko_tsdf Eigen3::Eigen)
  ament_target_dependencies(test_volumetric_mapper slamko_core)

  ament_add_gtest(test_tsdf_io test/test_io.cpp)
  target_link_libraries(test_tsdf_io slamko_tsdf Eigen3::Eigen)
  ament_target_dependencies(test_tsdf_io slamko_core)

  ament_add_gtest(test_live_driver test/test_live_driver.cpp)
  target_link_libraries(test_live_driver slamko_tsdf Eigen3::Eigen)
  ament_target_dependencies(test_live_driver slamko_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(slamko_tsdf)
ament_export_dependencies(slamko_core Eigen3)
ament_package()
