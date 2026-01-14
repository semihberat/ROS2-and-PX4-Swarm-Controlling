#ifndef SWARM_MEMBER_PATH_PLANNER_HPP
#define SWARM_MEMBER_PATH_PLANNER_HPP

// Standard Libraries
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <tuple>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <atomic>
#include <chrono>
#include <memory>
#include <cstdint>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Custom Interface for Neighbors Info
#include <custom_interfaces/msg/neighbors_info.hpp>
#include <custom_interfaces/msg/target_positions.hpp>

// Path Planning Formulations
#include "../../src/formulations/geographic/calculate_center_of_gravity.hpp"
#include "../../src/formulations/geographic/calculate_offset_from_center.hpp"
#include "../../src/formulations/geographic/calculate_distance.hpp"

// Interfaces
#include "../../src/interfaces/vehicle_positions.hpp"
#include "../../src/interfaces/vectoral_distance.hpp"

#include <px4_msgs/msg/trajectory_setpoint.hpp>

using namespace lifecycle_msgs::msg;
using namespace px4_msgs::msg;
using namespace custom_interfaces::msg;
using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace std::chrono;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SwarmMemberPathPlanner : public rclcpp_lifecycle::LifecycleNode
{
public:
    SwarmMemberPathPlanner();

private:
    // Timer callback function
    void timer_callback();

    // Subscribers
    rclcpp::Subscription<NeighborsInfo>::SharedPtr neighbors_info_subscription_;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Temporary variables
    VehicleVerticalPositions target_position_;
    float target_dlat, target_dlon;
    float target_vlat, target_vlon;
    unsigned int verification_count = 0;
    const unsigned int verification_count_max = 20;
    float offset_lat = 2.0f;
    float offset_lon = 2.0f;
    float desired_vel = 5.0f;
    std::mutex data_mutex_;

    // Lists
    std::vector<px4_msgs::msg::VehicleGlobalPosition> all_positions;

    // Safety Parameters
    const float collision_tolerance_m = sqrt(pow(offset_lat, 2) + pow(offset_lon, 2)) / 2.0f;

    // Callback functions
    void path_planner_callback(const NeighborsInfo::SharedPtr msg);

    // Publishers
    void publish_trajectory_setpoint(float x, float y, float z, float yaw_rad);

    // Lifecycle callbacks
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);
};

#endif // SWARM_MEMBER_PATH_PLANNER_HPP
