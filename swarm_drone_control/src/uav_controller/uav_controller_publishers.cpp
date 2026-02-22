#include "uav_controller.hpp"

/** @brief Send arming command to PX4 */
void UAVController::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/** @brief Send disarming command to PX4 */
void UAVController::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void UAVController::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->offboard_control_mode_publisher_->publish(msg);
}

void UAVController::publish_trajectory_setpoint(float x, float y, float z, float yaw_rad)
{
    TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {x, y, z};
    msg.yawspeed = yaw_rad;
    msg.yaw = NAN;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->trajectory_setpoint_publisher_->publish(msg);
}

void UAVController::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = this->sys_id + 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->vehicle_command_publisher_->publish(msg);
}
