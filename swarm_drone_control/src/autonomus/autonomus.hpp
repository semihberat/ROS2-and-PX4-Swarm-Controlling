#ifndef SWARM_MEMBER_PATH_PLANNER_HPP
#define SWARM_MEMBER_PATH_PLANNER_HPP

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
#include <custom_interfaces/msg/neighbors_info.hpp>
#include <custom_interfaces/msg/target_positions.hpp>
#include "calculations/geographic.hpp"
#include <px4_msgs/msg/trajectory_setpoint.hpp>

using namespace lifecycle_msgs::msg;
using namespace px4_msgs::msg;
using namespace custom_interfaces::msg;
using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace std::chrono;

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Autonomous swarm member that follows formation and executes missions
 */
class SwarmMemberPathPlanner : public rclcpp_lifecycle::LifecycleNode
{
public:
    SwarmMemberPathPlanner();

private:
    rclcpp::Subscription<NeighborsInfo>::SharedPtr neighbors_info_subscription_;
    rclcpp_lifecycle::LifecyclePublisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    LatLon target_position_;
    float target_dlat, target_dlon;
    float target_vlat, target_vlon;
    unsigned int verification_count = 0;
    const unsigned int verification_count_max = 20;
    float offset_lat = 2.0f;
    float offset_lon = 2.0f;
    float desired_vel = 5.0f;
    std::mutex data_mutex_;

    // Neighbors_ INfo
    NeighborsInfo::SharedPtr current_neighbors_info_;

    enum class Mission
    {
        FORMATIONAL_TAKEOFF,
        FORMATIONAL_ROTATION,
        GOTO_POSITION,
        DO_PROCESS,
        END_TASK
    } current_mission = Mission::FORMATIONAL_TAKEOFF;

    // Lists
    std::vector<px4_msgs::msg::VehicleGlobalPosition> all_positions;

    // Safety Parameters
    const float collision_tolerance_m = sqrt(pow(offset_lat, 2) + pow(offset_lon, 2)) / 2.0f;

    /**
     * @brief Send velocity commands to PX4
     * @param x North velocity (m/s)
     * @param y East velocity (m/s)
     * @param z Down velocity (m/s)
     * @param yaw_rad Yaw rate (rad/s)
     */
    void publish_trajectory_setpoint(float x, float y, float z, float yaw_rad);

    /** @brief Setup publishers, subscribers, and load parameters */
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);

    /** @brief Start timers and enable control */
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);

    /** @brief Stop control and timers */
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);

    /** @brief Release ROS resources */
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);

    /** @brief Graceful shutdown */
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

    /** @brief Handle error state */
    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);

    /** @brief Execute formational takeoff mission */
    void formational_takeoff();

    /** @brief Execute formational rotation mission */
    void formational_rotation();

    /** @brief Navigate to target position */
    void goto_position();

    /** @brief Execute custom process logic */
    void do_process();

    /** @brief Finalize and end current task */
    void end_task();

    /** @brief Advance to next mission step */
    void next_step();

    /** @brief Main state machine timer */
    void state_cycle_callback();

    /** @brief Update neighbor positions for swarm coordination */
    void neighbors_info_subscriber(const NeighborsInfo::SharedPtr msg);
};

#endif // SWARM_MEMBER_PATH_PLANNER_HPP
