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
#include <custom_interfaces/msg/neighbors_info.hpp>
#include <custom_interfaces/msg/waypoints.hpp>
#include <custom_interfaces/msg/in_target.hpp>
#include <custom_interfaces/msg/qr_information.hpp>
#include "calculations/geographic.hpp"
#include "calculations/spatial.hpp"
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

using namespace lifecycle_msgs::msg;
using namespace px4_msgs::msg;
using namespace custom_interfaces::msg;
using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace std::chrono;

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

#include "autonomous/autonomous_utils.hpp"

/**
 * @brief Autonomous swarm member that follows formation and executes missions
 */
class SwarmMemberPathPlanner : public rclcpp_lifecycle::LifecycleNode
{
public:
    SwarmMemberPathPlanner();

private:
    int sys_id_{1};
    int total_drones_{3};

    rclcpp::Subscription<NeighborsInfo>::SharedPtr neighbors_info_subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
    rclcpp_lifecycle::LifecyclePublisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    // Target synchronization components
    rclcpp_lifecycle::LifecyclePublisher<InTarget>::SharedPtr in_target_publisher_;
    std::vector<rclcpp::Subscription<InTarget>::SharedPtr> in_target_subs_;
    std::vector<bool> drones_in_target_;

    // QR Information Multi-Subscribing
    std::vector<rclcpp::Subscription<QRInformation>::SharedPtr> qr_subs_;
    QRInformation::SharedPtr qr_info_;
    QRInformation::SharedPtr latest_qr_info_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_2;

    LatLon target_position_;
    float target_dlat, target_dlon;
    float target_vlat, target_vlon;

    double target_altitude_ = -10.0;
    double current_altitude;
    const VehicleGlobalPosition *current_wp_ = nullptr;

    struct DesiredVelocities
    {
        float vel = 2.0;
        float z_vel = 2.0;
        float yaw_vel = 1.0;
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
        VehicleGlobalPosition leader_vehicle;
        VehicleGlobalPosition circular_position;
        VehicleGlobalPosition target_after_offset;
        DLatDLon offset_from_cog;
        double target_bearing_from_cog = 0.0;
        uint8_t leader_id = 0;
    } swarm_positions;

    DLatDLon target_distance_;

    rclcpp::CallbackGroup::SharedPtr cb_group;
    rclcpp::CallbackGroup::SharedPtr cb_group_2;

    // Neighbors info
    NeighborsInfo::SharedPtr current_neighbors_info_;
    spatial::EulerAngles current_euler_angles_;
    
    // Mission context
    bool home_position_stored = false;
    VehicleGlobalPosition initial_home_pos_;
    
    std::vector<DLatDLon> initial_n_distances;
    std::vector<DLatDLon> current_n_distances;

    struct CollisionAvoidanceParams
    {
        float k_repulsive = 1.5f;    // Güçlü itki (çarpışmayı kesin önlemek için)
        float k_attractive = 0.5f;   // Yumuşak çekiş (formasyondan kopmamak için)
        float k_vortex = 1.0f;       // Türbülans/Teğet kuvveti
        float safe_threshold = 0.4f; // Tolerans bandı (+/- 0.4 metre)
        float max_bias = 2.0f;       // Maksimum etki (hız limiti sapması)
        float filter_alpha = 0.7f;   // Low-pass filtre yumuşatması (0.7 eski, 0.3 yeni)
    } col_avoid_params;

    struct CollisionBias
    {
        float vlat = 0.0;
        float vlon = 0.0;
    } collision_bias;

    enum class Mission
    {
        FORMATIONAL_TAKEOFF,
        FORMATIONAL_ROTATION,
        GOTO_POSITION,
        DO_PROCESS,
        END_TASK
    } current_mission;

    enum class DoProcess
    {
        FORMATION,
        MANUEVER_PITCH_ROLL,
        ALTITUDE_CHANGE,
        LEAVE_THE_SWARM,
        NEXT
    } current_process;

    autonomous_utils::WaypointManager waypoint_manager_;

    // Lists
    std::vector<px4_msgs::msg::VehicleGlobalPosition> all_positions;

    void setup_publishers_and_subscribers();
    void setup_in_target_subscribers(const rclcpp::QoS &qos);
    void setup_qr_subscribers(const rclcpp::QoS &qos);
    void setup_timers();
    void reset_timers();
    void clear_pointers();

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

    /** @brief Modify commands with collision bias before publishing */
    void apply_collision_bias();

    /** @brief Update horizontal velocity commands based on target distance */
    void update_velocity();

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

    /** @brief Check and sync target arrivals */
    void in_target_callback(const InTarget::SharedPtr msg);
    bool check_all_drones_in_target();
    bool verify_in_target_state(double current_error, double target_threshold = autonomous_utils::STOP_THRESHOLD_01);
    void reset_in_target_status();

    /** @brief Process scanned QR Info from any drone in the swarm */
    void qr_callback(const QRInformation::SharedPtr msg);

    /** @brief Update vehicle attitude (Euler angles) */
    void vehicle_attitude_subscriber(const VehicleAttitude::SharedPtr msg);

    /** @brief Calculate initial values for mission */
    void initial_calculations_before_mission();

    // --- Modular Calculation Functions ---
    void calculate_swarm_positions();
    void elect_leader();
    void calculate_mission_specific_targets();

    // DO_PROCESS
    void formation(const custom_interfaces::msg::Formation &formasyon);
    void manuever_pitch_roll(const custom_interfaces::msg::PitchRollMovement &manevra);
    void altitude_change(const custom_interfaces::msg::AltitudeChange &irtifa_degisim);
    void leave_the_swarm(const custom_interfaces::msg::LeaveTheHerd &suruden_ayrilma);
};

#endif // SWARM_MEMBER_PATH_PLANNER_HPP
