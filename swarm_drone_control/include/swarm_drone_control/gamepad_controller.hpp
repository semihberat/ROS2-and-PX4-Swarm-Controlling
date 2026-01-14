#ifndef GAMEPAD_CONTROLLER_HPP
#define GAMEPAD_CONTROLLER_HPP

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <sensor_msgs/msg/joy.hpp>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std::placeholders;

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class OffboardControl : public rclcpp_lifecycle::LifecycleNode
{
public:
    OffboardControl();

    // Lifecycle callbacks
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

    void arm();
    void disarm();

private:
    rclcpp_lifecycle::LifecyclePublisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    uint64_t offboard_setpoint_counter_;
    sensor_msgs::msg::Joy::SharedPtr joy_msgs;
    px4_msgs::msg::VehicleAttitude::SharedPtr vehicle_attitude_;
    px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status_;

    void publish_trajectory_setpoint(float x, float y, float z, float yawspeed);
    void joy_subscriber_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void relative_movement(float v_x, float v_y, float v_z, float yawspeed);
    void listen_to_altitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void vehicle_status_listener_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
};

#endif // GAMEPAD_CONTROLLER_HPP
