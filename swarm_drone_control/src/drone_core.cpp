#include "drone_core/drone_core.hpp"

void DroneCore::create_timers()
{
	this->cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	timer_ = this->create_wall_timer(100ms, std::bind(&DroneCore::timer_callback, this), this->cb_group_);
}

void DroneCore::create_publishers()
{
	offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/px4_" + std::to_string(this->sys_id) + "/fmu/in/offboard_control_mode", 10);
	trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/px4_" + std::to_string(this->sys_id) + "/fmu/in/trajectory_setpoint", 10);
	vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/px4_" + std::to_string(this->sys_id) + "/fmu/in/vehicle_command", 10);
	drone_info_publisher_ = this->create_publisher<custom_interfaces::msg::DroneInfo>("/drone_info", 10);
}

void DroneCore::create_subscribers(rclcpp::QoS qos)
{
	this->cb_group_glob_pos_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	this->glob_pos_options_.callback_group = this->cb_group_glob_pos_;
	vehicle_global_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
		"/px4_" + std::to_string(this->sys_id) + "/fmu/out/vehicle_global_position",
		qos,
		std::bind(&DroneCore::listen_vehicle_global_position, this, _1),
		this->glob_pos_options_);

	this->cb_group_swarm_info_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	this->swarm_info_options_.callback_group = this->cb_group_swarm_info_;
	swarm_info_subscriber_ = this->create_subscription<custom_interfaces::msg::DroneInfo>(
		"/drone_info",
		qos,
		std::bind(&DroneCore::listen_swarm_info, this, _1),
		this->swarm_info_options_);

	this->cb_group_vehicle_ctrl_mode_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	this->vehicle_ctrl_mode_options_.callback_group = this->cb_group_vehicle_ctrl_mode_;
	vehicle_control_mode_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
		"/px4_" + std::to_string(this->sys_id) + "/fmu/out/vehicle_control_mode",
		qos,
		std::bind(&DroneCore::listen_vehicle_control_mode, this, _1),
		this->vehicle_ctrl_mode_options_);

	this->cb_group_vehicle_attitude_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	this->vehicle_attitude_options_.callback_group = this->cb_group_vehicle_attitude_;
	vehicle_attitude_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
		"/px4_" + std::to_string(this->sys_id) + "/fmu/out/vehicle_attitude",
		qos,
		std::bind(&DroneCore::listen_vehicle_attitude, this, _1),
		this->vehicle_attitude_options_);
}

void DroneCore::create_action_servers()
{
	this->cb_group_action_server_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	action_server_ = rclcpp_action::create_server<MoveDrone>(
		this,
		"/uav_" + std::to_string(this->sys_id) + "/move_drone",
		std::bind(&DroneCore::goal_callback, this, _1, _2),
		std::bind(&DroneCore::cancel_callback, this, _1),
		std::bind(&DroneCore::handle_accepted, this, _1),
		rcl_action_server_get_default_options(),
		this->cb_group_action_server_);
}

void DroneCore::create_service_servers()
{
	this->create_service<custom_interfaces::srv::SetParameters>(
		"/uav_" + std::to_string(this->sys_id) + "/set_parameters",
		std::bind(&DroneCore::callbackSetParameters, this, _1, _2));
}

void DroneCore::declare_all_parameters()
{
	this->declare_parameter("sys_id", 1);
	sys_id = this->get_parameter("sys_id").as_int();
}

DroneCore::DroneCore() : rclcpp::Node("drone_core")
{

	swarm_info_.reserve(24);
	swarm_info_.resize(24);

	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	drone_info_ = std::make_shared<custom_interfaces::msg::DroneInfo>();

	declare_all_parameters();
	create_timers();
	create_publishers();
	create_subscribers(qos);
	create_action_servers();
	offboard_setpoint_counter_ = 0;
}

void DroneCore::arm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void DroneCore::disarm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void DroneCore::publish_offboard_control_mode()
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void DroneCore::publish_trajectory_setpoint(float x, float y, float z, float yawspeed)
{
	px4_msgs::msg::TrajectorySetpoint msg{};

	msg.position = {NAN, NAN, NAN}; // set NAN to ignore position and velocity control
	msg.yaw = NAN;					// set NAN to ignore yaw control
	msg.velocity = {x, y, z};
	msg.yawspeed = yawspeed; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void DroneCore::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = this->sys_id + 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void DroneCore::timer_callback()
{
	if (offboard_setpoint_counter_ == 10)
	{
		this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	}

	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_offboard_control_mode();
	publish_current_position();

	// stop the counter after reaching 11
	if (offboard_setpoint_counter_ < 11)
	{
		offboard_setpoint_counter_++;
		publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
	}
}

void DroneCore::listen_vehicle_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
	drone_info_->geo_point.lat = msg->lat;
	drone_info_->geo_point.lon = msg->lon;
	drone_info_->geo_point.alt = msg->alt;
}

void DroneCore::publish_current_position()
{
	custom_interfaces::msg::DroneInfo msg{};
	msg.id = this->sys_id;
	msg.geo_point = drone_info_->geo_point;
	msg.geo_vel = drone_info_->geo_vel;
	drone_info_publisher_->publish(msg);
}

void DroneCore::listen_swarm_info(const custom_interfaces::msg::DroneInfo::SharedPtr msg)
{
	swarm_info_[msg->id - 1] = msg;
}

void DroneCore::listen_vehicle_control_mode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
{
	is_armed_ = msg->flag_armed;
}

void DroneCore::listen_vehicle_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
	drone_info_->yaw = atan2(2.0f * (msg->q[0] * msg->q[3] + msg->q[1] * msg->q[2]), 1.0f - 2.0f * (msg->q[2] * msg->q[2] + msg->q[3] * msg->q[3]));
}

void DroneCore::callbackSetParameters(const std::shared_ptr<custom_interfaces::srv::SetParameters::Request> request,
									  std::shared_ptr<custom_interfaces::srv::SetParameters::Response> response)
{
	if (request->id != this->sys_id || request->id != 0)
		return;
	service_parameters_.alt_vel = request->alt_vel;
	service_parameters_.formation_radius = request->formation_radius;
	service_parameters_.max_alt = request->max_alt;
	service_parameters_.min_alt = request->min_alt;
	service_parameters_.takeoff_alt = request->takeoff_alt;
	service_parameters_.velocity = request->velocity;
	service_parameters_.yaw_vel = request->yaw_vel;
	service_parameters_.takeoff_alt = request->takeoff_alt;

	response->success = true;
	response->message = "Parameters updated successfully.";
}
