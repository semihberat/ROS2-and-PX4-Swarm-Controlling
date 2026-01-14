#include "swarm_drone_control/uav_controller.hpp"

// === CONSTRUCTOR ===
UAVController::UAVController() : rclcpp::Node("uav_controller")
{
	this->declare_parameter("sys_id", 1);
	sys_id = this->get_parameter("sys_id").as_int();

	// Topic Names
	std::string OFFBOARD_CONTROL_MODE = "/px4_" + std::to_string(sys_id) + "/fmu/in/offboard_control_mode";
	std::string TRAJECTORY_SETPOINT = "/px4_" + std::to_string(sys_id) + "/fmu/in/trajectory_setpoint";
	std::string VEHICLE_COMMAND = "/px4_" + std::to_string(sys_id) + "/fmu/in/vehicle_command";

	// Publishers
	offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(OFFBOARD_CONTROL_MODE, 10);
	trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(TRAJECTORY_SETPOINT, 10);
	vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(VEHICLE_COMMAND, 10);
	offboard_setpoint_counter_ = 0;

	// Timer for publisher callback
	timer_ = this->create_wall_timer(100ms, std::bind(&UAVController::timer_callback, this));
}

// === TIMER CALLBACK ===
void UAVController::timer_callback()
{
	if (offboard_setpoint_counter_ == 10)
	{
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	}

	this->publish_offboard_control_mode();
	// stop the counter after reaching 11
	if (offboard_setpoint_counter_ < 11)
	{
		this->publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);

		offboard_setpoint_counter_++;
	}
}

// === PUBLISH FUNCTIONS ===
void UAVController::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

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
	offboard_control_mode_publisher_->publish(msg);
}

void UAVController::publish_trajectory_setpoint(float x, float y, float z, float yaw_rad)
{
	TrajectorySetpoint msg{};
	msg.position = {NAN, NAN, NAN};
	msg.velocity = {x, y, z};
	msg.yawspeed = yaw_rad;
	msg.yaw = NAN; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void UAVController::publish_vehicle_command(uint16_t command, float param1, float param2)
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

int main(int argc, char *argv[])
{
	std::cout << "============== UAV Controller ==============" << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UAVController>());
	rclcpp::shutdown();
	return 0;
}