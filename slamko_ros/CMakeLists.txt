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

find_package(ament_cmake REQUIRED)
find_package(slamko_core REQUIRED)
find_package(slamko_loop REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(provider_fusion_node nodes/provider_fusion_node.cpp)
target_link_libraries(provider_fusion_node Eigen3::Eigen)
ament_target_dependencies(provider_fusion_node
  slamko_core slamko_loop rclcpp nav_msgs geometry_msgs tf2_ros)
install(TARGETS provider_fusion_node RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(provider_chain_offline tools/provider_chain_offline.cpp)
target_link_libraries(provider_chain_offline Eigen3::Eigen)
ament_target_dependencies(provider_chain_offline slamko_core slamko_loop)
install(TARGETS provider_chain_offline RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
