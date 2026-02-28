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

namespace autonomus_utils
{
    constexpr double STOP_THRESHOLD_01 = 0.1;
    constexpr double STOP_THRESHOLD_001 = 0.01;

    inline bool NeighborVerification(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &neighbors,
                                     std::function<double(const px4_msgs::msg::VehicleGlobalPosition &)> func)
    {
        return std::all_of(neighbors.begin(), neighbors.end(), [func](const px4_msgs::msg::VehicleGlobalPosition &pos)
                           { return std::abs(func(pos)) <= STOP_THRESHOLD_01; });
    }

    inline std::vector<DLatDLon> all_distances(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &neighbors, const px4_msgs::msg::VehicleGlobalPosition &main_position)
    {
        std::vector<DLatDLon> distances;
        distances.reserve(neighbors.size());
        for (const auto &neighbor_pos : neighbors)
        {
            distances.push_back(geo::calculate_distance<DLatDLon>(main_position.lat, main_position.lon,
                                                                  neighbor_pos.lat, neighbor_pos.lon));
        }
        return distances;
    }
}

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

    double current_altitude;

    float desired_vel = 2.0;
    float desired_z_vel = 2.0;
    float desired_yaw_vel = 0.5;
    float desired_v_lat = 2.0;
    float desired_v_lon = 2.0;
    std::mutex data_mutex_;

    VehicleGlobalPosition cog;
    VehicleGlobalPosition nearest_vehicle;
    VehicleGlobalPosition circular_position;
    VehicleGlobalPosition target_after_offset;

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
    Waypoints::SharedPtr waypoints_;
    VehicleGlobalPosition current_waypoint_;
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

    /** @brief In target service callback */
    void in_target_callback(const InTarget::Request::SharedPtr request, const InTarget::Response::SharedPtr response);
    void in_target_client_callback(rclcpp::Client<InTarget>::SharedFuture future);
    void call_in_target_client();
};

#endif // SWARM_MEMBER_PATH_PLANNER_HPP
