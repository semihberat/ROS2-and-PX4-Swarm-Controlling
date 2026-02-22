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

/**
 * @brief Manual control node using gamepad/joystick for drone operation
 */
class GamepadController : public rclcpp_lifecycle::LifecycleNode
{
public:
    GamepadController();

    /** @brief Setup publishers, subscribers, load parameters */
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

    /** @brief Activate publishers and start control timer */
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    /** @brief Deactivate control and stop timers */
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    /** @brief Release ROS resources */
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    /** @brief Graceful shutdown */
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    /** @brief Handle error state */
    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

    /** @brief Send arm command */
    void arm();

    /** @brief Send disarm command */
    void disarm();

private:
    // Lifecycle publishers and subscribers
    rclcpp_lifecycle::LifecyclePublisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t offboard_setpoint_counter_;

    // Latest states
    sensor_msgs::msg::Joy::SharedPtr joystick_state_;
    px4_msgs::msg::VehicleAttitude::SharedPtr vehicle_attitude_;
    px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status_;

    /**
     * @brief Convert joystick to world-frame velocities
     * @param velocity_x Body-frame forward velocity
     * @param velocity_y Body-frame lateral velocity
     * @param velocity_z Vertical velocity
     * @param yawspeed Yaw rate
     */
    void relative_movement(float velocity_x, float velocity_y, float velocity_z, float yawspeed);

    /** @brief Handle joystick input events */
    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /** @brief Send command to PX4 (arm, disarm, mode change, etc.) */
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    /** @brief Send velocity setpoint to drone */
    void publish_trajectory_setpoint(float x, float y, float z, float yawspeed);

    /** @brief Update drone attitude (quaternion to yaw) */
    void altitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);

    /** @brief Monitor arming status */
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

    /** @brief Timer callback for periodic control updates */
    void controller_callback();
};

#endif // GAMEPAD_CONTROLLER_HPP
