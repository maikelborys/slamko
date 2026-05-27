cmake_minimum_required(VERSION 3.22)
project(slamko_fusion LANGUAGES CXX)

# slamko_fusion — the GTSAM Tier-2 smoother. GTSAM is hidden behind the PIMPL, so
# only this package compiles against it. Needs GTSAM 4.3 (4.2 ships
# IncrementalFixedLagSmoother under nonlinear_unstable/, 4.3 under nonlinear/).

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
# Force the 4.3 install at /usr/local — it has FixedLagSmoother in libgtsam (4.2 at
# /usr puts it in gtsam_unstable, and mixing 4.3 headers with the 4.2 lib breaks the
# link with undefined FixedLagSmoother vtables).
find_package(GTSAM REQUIRED PATHS /usr/local/lib/cmake/GTSAM NO_DEFAULT_PATH)
message(STATUS "slamko_fusion using GTSAM ${GTSAM_VERSION} from ${GTSAM_DIR}")

# /usr/local/lib has the 4.3 libgtsam; /usr has a 4.2 with the same SONAME. Use
# DT_RPATH (transitive) so the 4.3 lib loads for the whole dependency chain at
# runtime — DT_RUNPATH (the default) is not transitive and lets the 4.2 lib win.
set(CMAKE_BUILD_RPATH /usr/local/lib)
set(CMAKE_INSTALL_RPATH /usr/local/lib)
add_link_options(-Wl,--disable-new-dtags)

# SHARED so GTSAM 4.3 is fully resolved INSIDE libslamko_fusion.so (PIMPL fully
# encapsulated) — consumers link the .so and never need to find_package(GTSAM) or
# fight the static-archive link order. The DT_RPATH below travels with the .so so
# its own gtsam NEEDED resolves to 4.3 at load time.
add_library(slamko_fusion SHARED src/gtsam_local_smoother.cpp)
target_include_directories(slamko_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(slamko_fusion Eigen3::Eigen gtsam)
ament_target_dependencies(slamko_fusion slamko_core)
target_compile_features(slamko_fusion PUBLIC cxx_std_17)

install(TARGETS slamko_fusion
  EXPORT slamko_fusionTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gtsam_smoother test/test_gtsam_smoother.cpp)
  target_link_libraries(test_gtsam_smoother slamko_fusion Eigen3::Eigen gtsam)
  ament_target_dependencies(test_gtsam_smoother slamko_core)
endif()

ament_export_dependencies(slamko_core Eigen3)
ament_export_include_directories(include)
ament_export_libraries(slamko_fusion)
ament_package()
