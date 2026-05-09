#ifndef SWARM_DRONE_CONTROL__UAV_CONTROLLER_HPP_
#define SWARM_DRONE_CONTROL__UAV_CONTROLLER_HPP_

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <custom_interfaces/msg/drone_info.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <custom_interfaces/msg/formation.hpp>
#include <custom_interfaces/msg/geo_vel.hpp>
#include <custom_interfaces/srv/set_parameters.hpp>
#include <custom_interfaces/srv/drone_commands.hpp>
#include <calculations/geographic.hpp>

#include <calculations/PID.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include "fields2cover.h"

#define DEFAULT_VELOCITY 5.0f
#define DEFAULT_YAW_VEL 1.0f
#define DEFAULT_ALT_VEL 2.0f
#define P_GAIN 0.9f
#define DEFAULT_TAKEOFF_ALT 5.0f
#define DEFAULT_MIN_ALT 1.0f
#define DEFAULT_MAX_ALT 200.0f
#define DEFAULT_FORMATION_RADIUS 5.0f
#define MAX_SWARM_SIZE 24

using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;
using std::placeholders::_2;

class DroneCore : public rclcpp::Node
{
public:
    DroneCore();

    void arm();
    void disarm();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr state_cycle_timer_;
    rclcpp::TimerBase::SharedPtr control_loop_;
    std::atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_;
    uint8_t sys_id;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<custom_interfaces::msg::DroneInfo>::SharedPtr drone_info_publisher_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_subscriber_;
    rclcpp::Subscription<custom_interfaces::msg::DroneInfo>::SharedPtr swarm_info_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;

    rclcpp::Service<custom_interfaces::srv::SetParameters>::SharedPtr set_parameter_server_;
    rclcpp::Service<custom_interfaces::srv::DroneCommands>::SharedPtr drone_command_server_;
    std::shared_ptr<custom_interfaces::srv::DroneCommands::Request> req_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::CallbackGroup::SharedPtr cb_group_glob_pos_;
    rclcpp::CallbackGroup::SharedPtr cb_group_swarm_info_;

    rclcpp::CallbackGroup::SharedPtr cb_group_vehicle_ctrl_mode_;
    rclcpp::CallbackGroup::SharedPtr cb_group_vehicle_attitude_;
    rclcpp::CallbackGroup::SharedPtr cb_group_vehicle_land_detected_;
    rclcpp::CallbackGroup::SharedPtr cb_group_state_cycle_;
    rclcpp::CallbackGroup::SharedPtr cb_group_local_pos_;
    rclcpp::CallbackGroup::SharedPtr cb_group_drone_comm_;

    rclcpp::SubscriptionOptions glob_pos_options_;
    rclcpp::SubscriptionOptions swarm_info_options_;
    rclcpp::SubscriptionOptions vehicle_ctrl_mode_options_;
    rclcpp::SubscriptionOptions vehicle_attitude_options_;
    rclcpp::SubscriptionOptions vehicle_land_detected_options_;
    rclcpp::SubscriptionOptions local_pos_options_;

    custom_interfaces::msg::DroneInfo::SharedPtr drone_info_;
    bool is_armed_ = false;
    std::vector<custom_interfaces::msg::DroneInfo::SharedPtr> swarm_info_;

    rclcpp_action::GoalUUID preempted_goal_id_;
    std::mutex mutex_;

    custom_interfaces::msg::GeoVel vll;
    bool land_detected_;
    size_t i_p = 0;
    double target_alt;
    std::vector<geo::Distance> init_dists_;

    // Containers
    void create_subscribers(rclcpp::QoS qos);
    void declare_all_parameters();
    void create_service_servers();
    void create_publishers();
    void create_timers();

    // publishers
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float x, float y, float z, float yawspeed);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_current_position();

    // subscribers
    void listen_vehicle_control_mode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
    void listen_vehicle_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void listen_vehicle_land_detected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    void listen_vehicle_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void listen_vehicle_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void listen_swarm_info(const custom_interfaces::msg::DroneInfo::SharedPtr msg);

    // control loops
    void timer_callback();
    void state_cycle();

    // Functionalities
    void collision_avoidance();
    void takeoff(float altitude);
    void land();
    void hold();
    void goto_waypoints(const std::vector<custom_interfaces::msg::GeoPoint> &waypoints);

    PID altitude_controller_{
        1.15f,
        0.0f,
        0.0f,
        0.1f,
        0.065};

    PID horizontal_PID_{
        1.2f,
        0.0f,
        0.0f,
        0.1f,
        0.025f};

    // Öneri: Daha yumuşak ve az agresif tepkiler için
    std::vector<PID> swarm_PIDs_ = std::vector<PID>(MAX_SWARM_SIZE, PID(0.6f, 0.1f, 0.2f, 0.1f, 1.0f));

    void callbackSetParameters(const std::shared_ptr<custom_interfaces::srv::SetParameters::Request> request,
                               std::shared_ptr<custom_interfaces::srv::SetParameters::Response> response);

    void callbackDroneCommands(const std::shared_ptr<custom_interfaces::srv::DroneCommands::Request> request,
                               std::shared_ptr<custom_interfaces::srv::DroneCommands::Response> response);

    struct ServiceParameters
    {
        float takeoff_alt = DEFAULT_TAKEOFF_ALT;
        float min_alt = DEFAULT_MIN_ALT;
        float max_alt = DEFAULT_MAX_ALT;
        float formation_radius = DEFAULT_FORMATION_RADIUS;
        float velocity = DEFAULT_VELOCITY;
        float yaw_vel = DEFAULT_YAW_VEL;
        float alt_vel = DEFAULT_ALT_VEL;
    } service_parameters_;

    struct Force
    {

        double vlat;
        double vlon;

    } forces_;

    // Overladings for lat lon distances
    // Allows: GeoPoint - DroneCore
    friend geo::Distance operator-(const custom_interfaces::msg::GeoPoint &points, const DroneCore &drone)
    {
        return geo::diff_points(points, drone.drone_info_->geo_point);
    }
};

#endif // SWARM_DRONE_CONTROL__UAV_CONTROLLER_HPP_
