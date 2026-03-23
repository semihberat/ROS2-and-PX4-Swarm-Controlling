cmake_minimum_required(VERSION 3.5)
project(swarm_drone_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
#################
# Setup targets #
#################

include_directories(include 
	src/formulations/geographic 
	src/formulations/trajectory
	src/interfaces
	src/autonomous
	src/uav_controller
	src/communication
	src/controller
	src/ground_control
)

# UAV Controller
add_executable(uav_controller 
	src/uav_controller/uav_controller.cpp
	src/uav_controller/uav_controller_main.cpp
	src/uav_controller/uav_controller_publishers.cpp
	src/uav_controller/uav_controller_subscribers.cpp
	src/uav_controller/uav_controller_timers.cpp
)
ament_target_dependencies(uav_controller rclcpp px4_msgs custom_interfaces rclcpp_lifecycle)
install(TARGETS uav_controller DESTINATION lib/${PROJECT_NAME})

# Autonomous Swarm Member (Path Planner)
add_executable(swarm_member_path_planner 
        src/autonomous/autonomous.cpp
        src/autonomous/autonomous_missions.cpp
        src/autonomous/autonomous_helpers.cpp
        src/autonomous/autonomous_processes.cpp
        src/autonomous/autonomous_publishers.cpp
        src/autonomous/autonomous_subscribers.cpp
        src/autonomous/autonomous_timers.cpp
        src/autonomous/autonomous_col_avoid.cpp
)
ament_target_dependencies(swarm_member_path_planner rclcpp px4_msgs custom_interfaces sensor_msgs rclcpp_lifecycle)
install(TARGETS swarm_member_path_planner DESTINATION lib/${PROJECT_NAME})

# Swarm Communication
add_executable(swarm_communication 
	src/communication/communication.cpp
	src/communication/communication_main.cpp
	src/communication/communication_publishers.cpp
	src/communication/communication_subscribers.cpp
	src/communication/communication_timers.cpp
)
ament_target_dependencies(swarm_communication rclcpp px4_msgs custom_interfaces rclcpp_lifecycle)
install(TARGETS swarm_communication DESTINATION lib/${PROJECT_NAME})

# Change State - Ground Control
add_executable(change_state src/ground_control/change_state.cpp)
ament_target_dependencies(change_state rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs sensor_msgs)
install(TARGETS change_state DESTINATION lib/${PROJECT_NAME})

# Gamepad Controller
add_executable(gamepad_controller 
	src/controller/controller.cpp
	src/controller/controller_lifecycle.cpp
	src/controller/controller_listeners.cpp
	src/controller/controller_main.cpp
	src/controller/controller_publishers.cpp
	src/controller/controller_timers.cpp
)
ament_target_dependencies(gamepad_controller rclcpp px4_msgs custom_interfaces rclcpp_lifecycle sensor_msgs)
install(TARGETS gamepad_controller DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs custom_interfaces rclcpp_lifecycle std_msgs lifecycle_msgs)

ament_export_include_directories(include)

# Install header files
install(DIRECTORY include/calculations DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  swarm_drone_control/scripts/camera_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
