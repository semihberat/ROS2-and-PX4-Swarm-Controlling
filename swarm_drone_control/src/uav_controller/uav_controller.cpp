#include "uav_controller.hpp"

// Basic UAV controller for offboard mode testing
UAVController::UAVController() : rclcpp::Node("uav_controller")
{
	this->declare_parameter("sys_id", 1);
	this->sys_id = this->get_parameter("sys_id").as_int();

	std::string OFFBOARD_CONTROL_MODE = "/px4_" + std::to_string(this->sys_id) + "/fmu/in/offboard_control_mode";
	std::string TRAJECTORY_SETPOINT = "/px4_" + std::to_string(this->sys_id) + "/fmu/in/trajectory_setpoint";
	std::string VEHICLE_COMMAND = "/px4_" + std::to_string(this->sys_id) + "/fmu/in/vehicle_command";

	this->offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(OFFBOARD_CONTROL_MODE, 10);
	this->trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(TRAJECTORY_SETPOINT, 10);
	this->vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(VEHICLE_COMMAND, 10);
	this->offboard_setpoint_counter_ = 0;

	timer_ = this->create_wall_timer(100ms, [this]() { this->timer_callback(); });
}
