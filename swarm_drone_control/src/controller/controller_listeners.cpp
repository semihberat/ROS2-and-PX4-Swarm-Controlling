#include "controller.hpp"

void GamepadController::joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    this->joystick_state_ = msg;
}

void GamepadController::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    this->vehicle_status_ = msg;
}

void GamepadController::altitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    this->vehicle_attitude_ = msg;
}

void GamepadController::neighbors_info_callback(const custom_interfaces::msg::NeighborsInfo::SharedPtr msg)
{
    this->current_neighbors_info_ = msg;
}
