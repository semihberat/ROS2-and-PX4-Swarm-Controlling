#pragma once

#ifndef OFFBOARD_CONTROLLER_HPP
#define OFFBOARD_CONTROLLER_HPP

// PX4_MSGS PUBLICATIONS
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

// PX4_MSGS SUBSCRIPTIONS
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

//CUSTOM INTERFACES
#include <custom_interfaces/msg/neighbors_info.hpp>


#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std::placeholders;

class OffboardController : public rclcpp::Node
{
public:
	OffboardController() : Node("uav_controller")
	{
		this->declare_parameter("sys_id", 1);
		this->declare_parameter("number_of_drones", 1);
		sys_id = this->get_parameter("sys_id").as_int();
		number_of_drones = this->get_parameter("number_of_drones").as_int();

		std::string ocmptpc = "/px4_" + std::to_string(sys_id) + "/fmu/in/offboard_control_mode";
		std::string tsptpc = "/px4_" + std::to_string(sys_id) + "/fmu/in/trajectory_setpoint";
		std::string vctpc = "/px4_" + std::to_string(sys_id) + "/fmu/in/vehicle_command";

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(ocmptpc, 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(tsptpc, 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(vctpc, 10);

		offboard_setpoint_counter_ = 0;
	}
	void arm();
	void disarm();

protected:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<custom_interfaces::msg::NeighborsInfo>::SharedPtr neighbors_gps_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	uint8_t sys_id;
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	uint8_t number_of_drones;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z, float yaw_rad);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publish_gps_to_neighbors(custom_interfaces::msg::NeighborsInfo msg);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardController::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardController::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardController::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardController::publish_trajectory_setpoint(float x, float y, float z, float yaw_rad)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = yaw_rad; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardController::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = sys_id + 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}


#endif // OFFBOARD_CONTROLLER_HPP