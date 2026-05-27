cmake_minimum_required(VERSION 3.22)
project(slamko_loop LANGUAGES CXX)

# slamko_loop — Tier-3 never-lost supervisor. Pure C++/Eigen over slamko_core
# contracts only (Hard Rule #2) — no GTSAM, no ROS, no dense. v1 is closed-form
# (no nonlinear solver): the seal/branch/weld policy is exact SE3 algebra, which
# is what makes it deterministically unit-testable. The pose-graph solver lands
# in P2.5; relocalization (XFeat) in P3.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(slamko_core REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(slamko_loop SHARED
  src/submap_archive.cpp
  src/anchor_gate.cpp
  src/never_lost_supervisor.cpp
  src/pose_graph.cpp
  src/xfeat_relocalizer.cpp)
target_include_directories(slamko_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(slamko_loop Eigen3::Eigen)
ament_target_dependencies(slamko_loop slamko_core)
target_compile_features(slamko_loop PUBLIC cxx_std_17)

install(TARGETS slamko_loop
  EXPORT slamko_loopTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t test_supervisor test_relocalizer test_pose_graph test_stress)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} slamko_loop Eigen3::Eigen)
    ament_target_dependencies(${t} slamko_core)
  endforeach()
endif()

ament_export_dependencies(slamko_core Eigen3)
ament_export_include_directories(include)
ament_export_libraries(slamko_loop)
ament_package()
