cmake_minimum_required(VERSION 3.5)
project(swarm_drone_control)

###############
# Build setup #
###############

# Use C99 and C++14 unless already set by the workspace/toolchain.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable stricter warnings for GCC/Clang.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################
# Dependencies #
################

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

###########
# Targets #
###########

# Public headers for C++ targets.
include_directories(include)

# Main UAV controller node.
add_executable(drone_core
  src/main.cpp
  src/drone_core.cpp
  src/action_servers.cpp
)
ament_target_dependencies(drone_core rclcpp px4_msgs custom_interfaces rclcpp_lifecycle rclcpp_action)

# Install C++ executable.
install(TARGETS drone_core
  DESTINATION lib/${PROJECT_NAME}
)

####################
# Export and install #
####################

# Export include path and dependencies for downstream ROS packages.
ament_export_include_directories(include)
ament_export_dependencies(
  ament_cmake
  rclcpp
  rosidl_default_runtime
  eigen3_cmake_module
  Eigen3
  px4_msgs
  geometry_msgs
  sensor_msgs
  custom_interfaces
  rclcpp_lifecycle
  std_msgs
  lifecycle_msgs
)

ament_export_include_directories(include)

# Install launch/config assets.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

##########
# Python #
##########

# Install Python package and scripts.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  swarm_drone_control/scripts/camera_bridge.py
  swarm_drone_control/scripts/ground_control/main.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
