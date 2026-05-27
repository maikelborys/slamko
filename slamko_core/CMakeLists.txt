cmake_minimum_required(VERSION 3.22)
project(slamko_core LANGUAGES CXX)

# slamko_core is header-only (contracts + types + SE3 manifold). Eigen only —
# no ROS, CUDA, GTSAM or Ceres deps. This is the spine everything else builds on.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------- header-only INTERFACE lib
add_library(slamko_core INTERFACE)
target_include_directories(slamko_core INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(slamko_core INTERFACE Eigen3::Eigen)
target_compile_features(slamko_core INTERFACE cxx_std_17)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS slamko_core EXPORT export_slamko_core)

ament_export_targets(export_slamko_core)
ament_export_dependencies(Eigen3)

# --------------------------------------------------------------------- tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_se3 test/test_se3.cpp)
  target_link_libraries(test_se3 slamko_core)

  ament_add_gtest(test_node_key test/test_node_key.cpp)
  target_link_libraries(test_node_key slamko_core)

  ament_add_gtest(test_contracts test/test_contracts.cpp)
  target_link_libraries(test_contracts slamko_core)
endif()

ament_package()
