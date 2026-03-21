#ifndef UAV_CONTROLLER_HPP
#define UAV_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
using namespace std::placeholders;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**
 * @brief Simple test controller for basic offboard mode functionality
 */
class UAVController : public rclcpp::Node
{
public:
    UAVController();

private:
    void timer_callback();

    // Publishers and subscribers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // Parameters
    rclcpp::TimerBase::SharedPtr timer_;
    uint8_t sys_id;
    uint64_t offboard_setpoint_counter_;

    /** @brief Send arm command */
    void arm();

    /** @brief Send disarm command */
    void disarm();

    /** @brief Publish offboard mode state */
    void publish_offboard_control_mode();

    /** @brief Send trajectory setpoint to PX4 */
    void publish_trajectory_setpoint(float x, float y, float z, float yaw_rad);

    /** @brief Send generic vehicle command */
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

#endif // UAV_CONTROLLER_HPP
