#ifndef SWARM_DRONE_CONTROL__UAV_CONTROLLER_HPP_
#define SWARM_DRONE_CONTROL__UAV_CONTROLLER_HPP_

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

#include <custom_interfaces/msg/drone_info.hpp>
#include <custom_interfaces/action/move_drone.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <custom_interfaces/msg/formation.hpp>
#include <custom_interfaces/msg/geo_vel.hpp>
#include <custom_interfaces/srv/set_parameters.hpp>
#include <calculations/geographic.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

#define DEFAULT_VELOCITY 5.0f
#define DEFAULT_YAW_VEL 1.0f
#define DEFAULT_ALT_VEL 2.0f
#define P_GAIN 0.85f
#define DEFAULT_TAKEOFF_ALT 5.0f
#define DEFAULT_MIN_ALT 1.0f
#define DEFAULT_MAX_ALT 200.0f
#define DEFAULT_FORMATION_RADIUS 5.0f

using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;
using std::placeholders::_2;

using MoveDrone = custom_interfaces::action::MoveDrone;
using MoveDroneGoalHandle = rclcpp_action::ServerGoalHandle<MoveDrone>;

class DroneCore : public rclcpp::Node
{
public:
    DroneCore();

    void arm();
    void disarm();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr control_loop_;
    std::atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_;
    uint8_t sys_id;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Publisher<custom_interfaces::msg::DroneInfo>::SharedPtr drone_info_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_subscriber_;
    rclcpp::Subscription<custom_interfaces::msg::DroneInfo>::SharedPtr swarm_info_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;

    rclcpp::Service<custom_interfaces::srv::SetParameters>::SharedPtr set_parameter_server_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::CallbackGroup::SharedPtr cb_group_glob_pos_;
    rclcpp::CallbackGroup::SharedPtr cb_group_swarm_info_;
    rclcpp::CallbackGroup::SharedPtr cb_group_action_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_vehicle_ctrl_mode_;
    rclcpp::CallbackGroup::SharedPtr cb_group_vehicle_attitude_;
    rclcpp::SubscriptionOptions glob_pos_options_;
    rclcpp::SubscriptionOptions swarm_info_options_;
    rclcpp::SubscriptionOptions vehicle_ctrl_mode_options_;
    rclcpp::SubscriptionOptions vehicle_attitude_options_;

    custom_interfaces::msg::DroneInfo::SharedPtr drone_info_;
    bool is_armed_ = false;
    std::vector<custom_interfaces::msg::DroneInfo::SharedPtr> swarm_info_;

    rclcpp_action::Server<MoveDrone>::SharedPtr action_server_;
    std::shared_ptr<MoveDroneGoalHandle> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;
    std::mutex mutex_;

    custom_interfaces::msg::GeoVel vll;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float x, float y, float z, float yawspeed);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void timer_callback();
    void create_publishers();
    void create_subscribers(rclcpp::QoS qos);
    void create_service_servers();
    void create_timers();
    void process_waypoints(std::shared_ptr<MoveDroneGoalHandle> goal_handle, const std::vector<custom_interfaces::msg::GeoPoint> &points);
    void listen_vehicle_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void listen_swarm_info(const custom_interfaces::msg::DroneInfo::SharedPtr msg);
    void publish_current_position();
    void control_loop_callback();
    void declare_all_parameters();
    void create_action_servers();
    void listen_vehicle_control_mode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
    void listen_vehicle_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);

    void callbackSetParameters(const std::shared_ptr<custom_interfaces::srv::SetParameters::Request> request,
                               std::shared_ptr<custom_interfaces::srv::SetParameters::Response> response);

    void execute_goal(const std::shared_ptr<MoveDroneGoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<MoveDroneGoalHandle> goal_handle);
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<MoveDroneGoalHandle> goal_handle);
    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,
                                              std::shared_ptr<const MoveDrone::Goal> goal);

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

    // Overladings for lat lon distances
    // Allows: GeoPoint - DroneCore
    friend geo::Distance operator-(const custom_interfaces::msg::GeoPoint &points, const DroneCore &drone)
    {
        return geo::diff_points(points, drone.drone_info_->geo_point);
    }
};

#endif // SWARM_DRONE_CONTROL__UAV_CONTROLLER_HPP_
