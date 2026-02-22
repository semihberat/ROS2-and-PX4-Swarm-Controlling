#include "controller.hpp"

/** @brief Send arming command to drone */
void GamepadController::arm()
{
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/** @brief Send disarming command to drone */
void GamepadController::disarm()
{
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Convert joystick commands to world frame velocities
 * @param velocity_x Body-frame forward velocity
 * @param velocity_y Body-frame lateral velocity
 * @param velocity_z Vertical velocity
 * @param yawspeed Yaw rate
 */
void GamepadController::relative_movement(float velocity_x, float velocity_y, float velocity_z, float yawspeed)
{
    // @note Extract yaw from quaternion using atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2))
    float yaw_angle = atan2f(2.0f * (this->vehicle_attitude_->q[0] * this->vehicle_attitude_->q[3] + this->vehicle_attitude_->q[1] * this->vehicle_attitude_->q[2]),
                             1.0f - 2.0f * (this->vehicle_attitude_->q[2] * this->vehicle_attitude_->q[2] + this->vehicle_attitude_->q[3] * this->vehicle_attitude_->q[3]));

    // Transform body-frame velocities to NED world frame
    float velocity_north = velocity_x * cosf(yaw_angle) - velocity_y * sinf(yaw_angle);
    float velocity_east = velocity_x * sinf(yaw_angle) + velocity_y * cosf(yaw_angle);

    this->publish_trajectory_setpoint(velocity_north, velocity_east, velocity_z, yawspeed);
}

void GamepadController::publish_trajectory_setpoint(float x, float y, float z, float yawspeed)
{
    TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {x, y, z};
    msg.yaw = NAN;
    msg.yawspeed = yawspeed;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->trajectory_setpoint_publisher_->publish(msg);
}

void GamepadController::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = this->get_parameter("sys_id").as_int() + 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->vehicle_command_publisher_->publish(msg);
}
