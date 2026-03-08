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
#include <algorithm>
#include <queue>
#include <functional>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <custom_interfaces/msg/neighbors_info.hpp>
#include <custom_interfaces/msg/target_positions.hpp>
#include <custom_interfaces/msg/waypoints.hpp>
#include <custom_interfaces/msg/nav_point.hpp>
#include "calculations/geographic.hpp"
#include "calculations/spatial.hpp"
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <custom_interfaces/srv/in_target.hpp>

using namespace lifecycle_msgs::msg;
using namespace px4_msgs::msg;
using namespace custom_interfaces::msg;
using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace std::chrono;
using namespace custom_interfaces::srv;

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ANSI Color Codes for Terminal Output
#define COLOR_RESET "\033[0m"
#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_MAGENTA "\033[1;35m"
#define COLOR_CYAN "\033[1;36m"
#define COLOR_WHITE "\033[1;37m"

// Colored Logging Macros
#define LOG_INFO(logger, ...) RCLCPP_INFO(logger, COLOR_CYAN __VA_ARGS__ COLOR_RESET)
#define LOG_WARN(logger, ...) RCLCPP_WARN(logger, COLOR_YELLOW __VA_ARGS__ COLOR_RESET)
#define LOG_ERROR(logger, ...) RCLCPP_ERROR(logger, COLOR_RED __VA_ARGS__ COLOR_RESET)
#define LOG_MISSION(logger, ...) RCLCPP_INFO(logger, COLOR_GREEN __VA_ARGS__ COLOR_RESET)
#define LOG_SUCCESS(logger, ...) RCLCPP_INFO(logger, COLOR_MAGENTA __VA_ARGS__ COLOR_RESET)

#include "autonomus_utils.hpp"

/**
 * @brief Autonomous swarm member that follows formation and executes missions
 */
class SwarmMemberPathPlanner : public rclcpp_lifecycle::LifecycleNode
{
public:
    SwarmMemberPathPlanner();

private:
    rclcpp::Subscription<NeighborsInfo>::SharedPtr neighbors_info_subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;

    rclcpp_lifecycle::LifecyclePublisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Service<InTarget>::SharedPtr in_target_service_;
    rclcpp::Client<InTarget>::SharedPtr in_target_client_;
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_2;

    LatLon target_position_;
    float target_dlat, target_dlon;
    float target_vlat, target_vlon;

    double target_altitude_ = -10.0;
    double current_altitude;

    struct DesiredVelocities
    {
        float vel = 2.0;
        float z_vel = 2.0;
        float yaw_vel = 0.5;
        float v_lat = 2.0;
        float v_lon = 2.0;
    } desired_velocities;

    struct CurrentCommands
    {
        double v_lat = 0.0;
        double v_lon = 0.0;
        double z_vel = 0.0;
        double yaw_vel = 0.0;
        double bearing = 0.0;
    } current_commands;

    std::mutex data_mutex_;

    struct SwarmPositions
    {
        VehicleGlobalPosition cog;
        VehicleGlobalPosition nearest_vehicle;
        VehicleGlobalPosition circular_position;
        VehicleGlobalPosition target_after_offset;
    } swarm_positions;

    rclcpp::CallbackGroup::SharedPtr cb_group;
    rclcpp::CallbackGroup::SharedPtr cb_group_2;

    // Neighbors info
    NeighborsInfo::SharedPtr current_neighbors_info_;
    spatial::EulerAngles current_euler_angles_;

    std::vector<DLatDLon> initial_n_distances;
    std::vector<DLatDLon> current_n_distances;
    std::vector<InTarget::Request::SharedPtr> positioned_drones_;

    enum class Mission
    {
        FORMATIONAL_TAKEOFF,
        FORMATIONAL_ROTATION,
        GOTO_POSITION,
        DO_PROCESS,
        END_TASK
    } current_mission;

    struct CollisionBias
    {
        float vlat = 0.0;
        float vlon = 0.0;
    } collision_bias;

    // Test base
    autonomus_utils::WaypointManager waypoint_manager_;

    // Lists
    std::vector<px4_msgs::msg::VehicleGlobalPosition>
        all_positions;

    // Safety Parameters

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

    /** @brief PARALLEL TIMERS */
    void state_cycle_callback();
    void collision_avoidance();

    /** @brief Update neighbor positions for swarm coordination */
    void neighbors_info_subscriber(const NeighborsInfo::SharedPtr msg);

    /** @brief Update vehicle attitude (Euler angles) */
    void vehicle_attitude_subscriber(const VehicleAttitude::SharedPtr msg);

    /** @brief Calculate initial values for mission */
    void initial_calculations_before_mission();

    /** @brief Initialize positioned drones vector for synchronization */
    void initialize_positioned_drones();

    /** @brief In target service callback */
    void in_target_callback(const InTarget::Request::SharedPtr request, const InTarget::Response::SharedPtr response);
    void in_target_client_callback(rclcpp::Client<InTarget>::SharedFuture future);
    void call_in_target_client();
};

#endif // SWARM_MEMBER_PATH_PLANNER_HPP
