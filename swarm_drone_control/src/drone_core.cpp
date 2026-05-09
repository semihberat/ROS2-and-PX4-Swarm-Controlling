#include "drone_core/drone_core.hpp"

DroneCore::DroneCore() : rclcpp::Node("drone_core")
{

	swarm_info_.reserve(MAX_SWARM_SIZE);
	swarm_info_.resize(MAX_SWARM_SIZE);
	init_dists_.reserve(MAX_SWARM_SIZE);
	init_dists_.resize(MAX_SWARM_SIZE);

	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	this->req_ = std::make_shared<custom_interfaces::srv::DroneCommands::Request>();
	this->req_->command = custom_interfaces::srv::DroneCommands::Request::HOLD;

	drone_info_ = std::make_shared<custom_interfaces::msg::DroneInfo>();

	declare_all_parameters();
	create_timers();
	create_publishers();
	create_subscribers(qos);
	create_service_servers();
	offboard_setpoint_counter_ = 0;
}

void DroneCore::create_timers()
{
	this->cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	timer_ = this->create_wall_timer(100ms, std::bind(&DroneCore::timer_callback, this), this->cb_group_);

	this->cb_group_state_cycle_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	state_cycle_timer_ = this->create_wall_timer(100ms, std::bind(&DroneCore::state_cycle, this), this->cb_group_state_cycle_);
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

	this->cb_group_vehicle_land_detected_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	this->vehicle_land_detected_options_.callback_group = this->cb_group_vehicle_land_detected_;
	vehicle_land_detected_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
		"/px4_" + std::to_string(this->sys_id) + "/fmu/out/vehicle_land_detected",
		qos,
		std::bind(&DroneCore::listen_vehicle_land_detected, this, _1),
		this->vehicle_land_detected_options_);

	this->cb_group_local_pos_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	this->local_pos_options_.callback_group = this->cb_group_local_pos_;
	vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		"/px4_" + std::to_string(this->sys_id) + "/fmu/out/vehicle_local_position_v1",
		qos,
		std::bind(&DroneCore::listen_vehicle_local_position, this, _1),
		this->local_pos_options_);
}

void DroneCore::create_service_servers()
{
	set_parameter_server_ = this->create_service<custom_interfaces::srv::SetParameters>(
		"/set_parameters",
		std::bind(&DroneCore::callbackSetParameters, this, _1, _2));

	this->cb_group_drone_comm_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	drone_command_server_ = this->create_service<custom_interfaces::srv::DroneCommands>(
		"/drone_commands",
		std::bind(&DroneCore::callbackDroneCommands, this, _1, _2),
		rmw_qos_profile_services_default,
		this->cb_group_drone_comm_);
}

void DroneCore::declare_all_parameters()
{
	this->declare_parameter("sys_id", 1);
	sys_id = this->get_parameter("sys_id").as_int();
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

	collision_avoidance();

	// stop the counter after reaching 11
	if (offboard_setpoint_counter_ < 11)
	{
		offboard_setpoint_counter_++;
		publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
	}
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

void DroneCore::listen_vehicle_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
	drone_info_->geo_point.lat = msg->lat;
	drone_info_->geo_point.lon = msg->lon;
}

void DroneCore::callbackSetParameters(const std::shared_ptr<custom_interfaces::srv::SetParameters::Request> request,
									  std::shared_ptr<custom_interfaces::srv::SetParameters::Response> response)
{
	if (request->id != this->sys_id || request->id != 0)
		return;
	service_parameters_.alt_vel = request->alt_vel;
	service_parameters_.max_alt = request->max_alt;
	service_parameters_.min_alt = request->min_alt;
	service_parameters_.takeoff_alt = request->takeoff_alt;
	service_parameters_.velocity = request->velocity;
	service_parameters_.yaw_vel = request->yaw_vel;
	service_parameters_.takeoff_alt = request->takeoff_alt;

	response->success = true;
	response->message = "[SUCCESS][PARAMETER_SERVICE] Parameters updated successfully.";
}

void DroneCore::callbackDroneCommands(const std::shared_ptr<custom_interfaces::srv::DroneCommands::Request> request,
									  std::shared_ptr<custom_interfaces::srv::DroneCommands::Response> response)
{
	const bool targets_this_drone = std::any_of(request->ids.begin(), request->ids.end(), [this](uint8_t id)
												{ return id == this->sys_id || id == 0; });
	if (!targets_this_drone)
		return;
	this->req_ = request;

	this->publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f); // stop the drone before executing the command
	switch (request->command)
	{
	case custom_interfaces::srv::DroneCommands::Request::HOLD:
		response->message = "[SUCCESS][HOLD] command received.";
		response->success = true;
		break;
	case custom_interfaces::srv::DroneCommands::Request::TAKEOFF:
		response->message = "[SUCCESS][TAKEOFF] command received.";
		response->success = true;
		break;
	case custom_interfaces::srv::DroneCommands::Request::LAND:
		// Implement LAND command logic here
		response->message = "[SUCCESS][LAND] command received.";
		response->success = true;
		break;
	case custom_interfaces::srv::DroneCommands::Request::WAYPOINT:

		if (!is_armed_)
		{
			RCLCPP_INFO(this->get_logger(), "Drone is not armed!");
			this->req_->command = custom_interfaces::srv::DroneCommands::Request::HOLD;
			response->message = "[ERROR][WAYPOINT] Drone is not armed.";
			response->success = false;
			return;
		};
		response->message = "[SUCCESS][WAYPOINT] command received.";
		response->success = true;
		break;
	case custom_interfaces::srv::DroneCommands::Request::GRID:
		response->message = "[SUCCESS][GRID] command received.";
		response->success = true;
		break;
	default:
		response->message = "[ERROR] Unknown command received.";
		response->success = false;
		break;
	}
}

void DroneCore::listen_vehicle_land_detected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	this->land_detected_ = msg->landed;
}

void DroneCore::listen_vehicle_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
	this->drone_info_->geo_point.alt = msg->z;
}

void DroneCore::collision_avoidance()
{
	using namespace custom_interfaces::srv;
	std::lock_guard<std::mutex> lock(mutex_);
	forces_.vlat = 0.0;
	forces_.vlon = 0.0;

	for (const auto &neighbor : this->swarm_info_)
	{
		if (neighbor == nullptr || neighbor->id == this->sys_id)
			continue;

		geo::Distance dist = neighbor->geo_point - *this;

		if (this->req_->command == DroneCommands::Request::TAKEOFF ||
			this->req_->command == DroneCommands::Request::HOLD)
		{
			init_dists_[neighbor->id - 1] = dist;
			continue;
		}

		swarm_PIDs_[neighbor->id - 1].setMaxOutput(1.0);
		swarm_PIDs_[neighbor->id - 1].setSetpoint(0.0);
		// Calculate desired relative displacement vs current displacement
		double error_lat = init_dists_[neighbor->id - 1].d_lat - dist.d_lat;
		double error_lon = init_dists_[neighbor->id - 1].d_lon - dist.d_lon;

		// Use P-control logic to calculate restoration forces
		// Gain value should be tuned, using 1.0 as a starting point
		forces_.vlat += swarm_PIDs_[neighbor->id - 1].update(error_lat);
		forces_.vlon += swarm_PIDs_[neighbor->id - 1].update(error_lon);
	}
}

void DroneCore::takeoff(float altitude)
{
	if (!is_armed_)
		this->arm();

	altitude_controller_.setMaxOutput(service_parameters_.alt_vel);
	altitude_controller_.setSetpoint(-altitude);
	float z_vel = altitude_controller_.update(this->drone_info_->geo_point.alt);
	this->publish_trajectory_setpoint(0.0f, 0.0f, z_vel, 0.0f);
}

void DroneCore::land()
{
	publish_trajectory_setpoint(0.0f, 0.0f, 1.0f, 0.0f);
	if (land_detected_)
	{
		this->disarm();
		if (!is_armed_)
		{
			this->req_->command = custom_interfaces::srv::DroneCommands::Request::HOLD;
			return;
		}
	}
}

void DroneCore::hold()
{
	if (this->is_armed_ && this->land_detected_)
	{
		this->disarm();
	}
	else
	{
		publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
	}
}

void DroneCore::goto_waypoints(const std::vector<custom_interfaces::msg::GeoPoint> &waypoints)
{
	geo::Distance dist = waypoints[i_p] - *this;
	double bearing = geo::calculate_bearing(this->drone_info_->geo_point, waypoints[i_p]);
	horizontal_PID_.setMaxOutput(service_parameters_.velocity);

	horizontal_PID_.setSetpoint(0.0);

	float vel = horizontal_PID_.update(-dist.d);
	float lat_vel = vel * std::cos(bearing);
	float lon_vel = vel * std::sin(bearing);

	{
		std::lock_guard<std::mutex> lock(mutex_);
		lat_vel -= forces_.vlat;
		lon_vel -= forces_.vlon;
	}
	// Print lat velocity and lon velocity
	RCLCPP_INFO(this->get_logger(), "Lat Vel: %f, Lon Vel: %f", lat_vel, lon_vel);
	double diff = bearing - this->drone_info_->yaw;
	if (diff > M_PI)
		diff -= 2 * M_PI;
	else if (diff < -M_PI)
		diff += 2 * M_PI;
	float yaw_vel = diff * service_parameters_.yaw_vel;
	this->publish_trajectory_setpoint(lat_vel, lon_vel, 0.0f, yaw_vel);
	if (dist.d <= 0.25)
	{

		if (i_p >= waypoints.size())
		{
			this->req_->command = custom_interfaces::srv::DroneCommands::Request::HOLD;
			return;
		}
	}
}
