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

# LighterGlue verifier (TorchScript via libtorch). OFF by default so the core
# build + the gtest suite stay torch-free; turn ON for the relocalization stack:
#   colcon build --packages-select slamko_loop \
#     --cmake-args -DSLAMKO_LOOP_WITH_TORCH=ON
# The .cpp is always compiled — without the macro it is a working no-op so the
# relocalizer transparently falls back to brute-force NN.
option(SLAMKO_LOOP_WITH_TORCH "Build the LighterGlue libtorch matcher" OFF)

# GTSAM pose-graph backends (PoseGraphBackend::GtsamLM batch + GtsamISAM2 incremental) — the
# MASTER_PLAN P-C′ solver. AUTO-DETECTED: ON when GTSAM is installed (it IS the planned backend, and
# iSAM2 is the live default), gracefully Ceres-only when absent (pose_graph_gtsam.cpp compiles as a
# stub that falls back to Ceres). Force Ceres-only with -DSLAMKO_LOOP_WITH_GTSAM=OFF.
option(SLAMKO_LOOP_WITH_GTSAM "Build the GTSAM backends (auto: ON if GTSAM found; OFF forces Ceres)" ON)

find_package(ament_cmake REQUIRED)
find_package(slamko_core REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
# Ceres backs the thin global pose-graph (src/pose_graph.cpp) — the P2.5 solver,
# rebuilt clean on branch klt-fork-loopclosure. Encapsulated in the .cpp; the
# public header (pose_graph.hpp) stays solver-free (Hard Rule #2).
find_package(Ceres REQUIRED)

# Relocalizer front-end (place-rec + verify) + the thin global PoseGraph. The old
# never-lost SUPERVISOR / anchor layer / SessionGraph were deleted 2026-05-29 (dead on
# the live path). pose_graph.cpp is the clean "loop closure + optimization, separate"
# layer: SE3 nodes + relative odometry edges + robust loop edges (Ceres).
add_library(slamko_loop SHARED
  src/bow.cpp
  src/xfeat_relocalizer.cpp
  src/lightglue_matcher.cpp
  src/pose_graph.cpp
  src/pose_graph_gtsam.cpp)
target_include_directories(slamko_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(slamko_loop Eigen3::Eigen Ceres::ceres)
ament_target_dependencies(slamko_loop slamko_core)
target_compile_features(slamko_loop PUBLIC cxx_std_17)

set(SLAMKO_GTSAM_ENABLED FALSE)
if(SLAMKO_LOOP_WITH_GTSAM)
  find_package(GTSAM QUIET)        # auto-detect: enable when present, Ceres-only when not
  if(GTSAM_FOUND)
    set(SLAMKO_GTSAM_ENABLED TRUE)
    target_compile_definitions(slamko_loop PRIVATE SLAMKO_HAVE_GTSAM)
    target_link_libraries(slamko_loop gtsam)
    message(STATUS "slamko_loop: GTSAM backends ENABLED (${GTSAM_VERSION}) — iSAM2 available")
  else()
    message(STATUS "slamko_loop: GTSAM not found — Ceres-only (set -DSLAMKO_LOOP_WITH_GTSAM=OFF to silence)")
  endif()
endif()

if(SLAMKO_LOOP_WITH_TORCH)
  find_package(Torch REQUIRED HINTS /home/maikel/libtorch $ENV{HOME}/libtorch)
  target_compile_definitions(slamko_loop PRIVATE SLAMKO_HAVE_TORCH)
  target_link_libraries(slamko_loop ${TORCH_LIBRARIES})
  target_include_directories(slamko_loop PRIVATE ${TORCH_INCLUDE_DIRS})
  message(STATUS "slamko_loop: LighterGlue matcher ENABLED (libtorch ${Torch_VERSION})")
endif()

install(TARGETS slamko_loop
  EXPORT slamko_loopTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# Tools: smap_info (archive inspector / VPR-coverage hard gate). The offline
# loop-closure driver (loop_offline) lands next to it in Milestone 2.
add_executable(smap_info tools/smap_info.cpp)
target_link_libraries(smap_info slamko_loop Eigen3::Eigen)
ament_target_dependencies(smap_info slamko_core)
install(TARGETS smap_info RUNTIME DESTINATION lib/${PROJECT_NAME})

# loop_offline — the offline loop-closure + optimization driver (Milestone 2).
add_executable(loop_offline tools/loop_offline.cpp)
target_link_libraries(loop_offline slamko_loop Eigen3::Eigen Ceres::ceres)
ament_target_dependencies(loop_offline slamko_core)
install(TARGETS loop_offline RUNTIME DESTINATION lib/${PROJECT_NAME})

# pose_graph_tum_ab — solver A/B on a real EuRoC trajectory (odom.tum + gt.tum -> ATE).
add_executable(pose_graph_tum_ab tools/pose_graph_tum_ab.cpp)
target_link_libraries(pose_graph_tum_ab slamko_loop Eigen3::Eigen Ceres::ceres)
ament_target_dependencies(pose_graph_tum_ab slamko_core)
install(TARGETS pose_graph_tum_ab RUNTIME DESTINATION lib/${PROJECT_NAME})

# vpr_recall_diag — P-B per-KF VPR recall diagnostic (start-room DB vs return
# queries vs full-map distractors; R@k + margins + per-query CSV).
add_executable(vpr_recall_diag tools/vpr_recall_diag.cpp)
target_link_libraries(vpr_recall_diag slamko_loop Eigen3::Eigen)
ament_target_dependencies(vpr_recall_diag slamko_core)
install(TARGETS vpr_recall_diag RUNTIME DESTINATION lib/${PROJECT_NAME})

# smap_cloud — export the global point cloud (anchor·local) for visualization.
add_executable(smap_cloud tools/smap_cloud.cpp)
target_link_libraries(smap_cloud slamko_loop Eigen3::Eigen)
ament_target_dependencies(smap_cloud slamko_core)
install(TARGETS smap_cloud RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t test_relocalizer test_bow test_pose_graph test_mappoint_store test_scan_context test_sdf_registration test_imu_shock)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} slamko_loop Eigen3::Eigen)
    ament_target_dependencies(${t} slamko_core)
  endforeach()
  if(SLAMKO_LOOP_WITH_TORCH)
    ament_add_gtest(test_lightglue test/test_lightglue.cpp)
    target_link_libraries(test_lightglue slamko_loop Eigen3::Eigen)
    ament_target_dependencies(test_lightglue slamko_core)
  endif()
  if(SLAMKO_GTSAM_ENABLED)
    ament_add_gtest(test_pose_graph_gtsam test/test_pose_graph_gtsam.cpp)
    target_link_libraries(test_pose_graph_gtsam slamko_loop Eigen3::Eigen)
    ament_target_dependencies(test_pose_graph_gtsam slamko_core)
  endif()
endif()

ament_export_dependencies(slamko_core Eigen3)
ament_export_include_directories(include)
ament_export_libraries(slamko_loop)
ament_package()
