#include <rclcpp/rclcpp.hpp>

// ========== PX4_MSGS ==========

// PX4_MSGS SUBSCRIPTIONS
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

// PX4_MSGS PUBLICATIONS
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <px4_msgs/msg/goto_setpoint.hpp>

// CUSTOM INTERFACES
#include <custom_interfaces/msg/neighbors_info.hpp>

// isimlendirme yaparken basina / koymak ros2'de namespace kullaniminda daha kati olmasi icin onlem aliyor
// eger / koymazsaniz node ismi namespace'in basina ekleniyor ve bu bazen istenmeyen durumlara yol acabiliyor
// ornegin node ismi offboard_control iken namespace /my_ns ise node ismi /my_ns/offboard_control oluyor

using namespace std::placeholders;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace custom_interfaces::msg;

class UAVController : public rclcpp::Node
{
public:
	UAVController() : rclcpp::Node("uav_controller")
	{
		this->declare_parameter("sys_id", 1);
		sys_id = this->get_parameter("sys_id").as_int();

		// Listener for GPS and Local Position
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		// Topic Names
		std::string OFFBOARD_CONTROL_MODE = "/px4_" + std::to_string(sys_id) + "/fmu/in/offboard_control_mode";
		std::string TRAJECTORY_SETPOINT = "/px4_" + std::to_string(sys_id) + "/fmu/in/trajectory_setpoint";
		std::string VEHICLE_COMMAND = "/px4_" + std::to_string(sys_id) + "/fmu/in/vehicle_command";
		std::string VEHICLE_GLOBAL_POSITION = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_global_position";
		std::string VEHICLE_LOCAL_POSITION = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_local_position_v1";
		

		// Subscribers

		vehicle_gps_subscriptions_ = this->create_subscription<VehicleGlobalPosition>(VEHICLE_GLOBAL_POSITION, qos,
																					  std::bind(&UAVController::gps_callback, this, _1));
		local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(VEHICLE_LOCAL_POSITION, qos,
																					   std::bind(&UAVController::local_position_callback, this, _1));

		// Publishers
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(OFFBOARD_CONTROL_MODE, 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(TRAJECTORY_SETPOINT, 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(VEHICLE_COMMAND, 10);
		offboard_setpoint_counter_ = 0;

		// Timer for publisher callback
		timer_ = this->create_wall_timer(100ms, std::bind(&UAVController::timer_callback, this));
	}
	void arm();
	void disarm();

	VehicleLocalPosition vehicle_local_position_;
	VehicleGlobalPosition vehicle_gps_position_;
	// Here is main function where the publisher and subscriber nodes are created and initialized.
private:
	// Subscribers
	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_gps_subscriptions_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;


	// Publishers
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	// Data Queues
	std::vector<VehicleGlobalPosition> neighbor_gps_queue_;
	std::vector<uint8_t> neighbor_id_queue_;

	// Parameters
	rclcpp::TimerBase::SharedPtr timer_;
	uint8_t sys_id;
	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

	// === Main Timer Callback ===
	void timer_callback();

	// === SUBSCRIBER CALLBACKS ===
	void gps_callback(const VehicleGlobalPosition::SharedPtr msg);
	void local_position_callback(const VehicleLocalPosition::SharedPtr msg);

	// === PUBLISH FUNCTIONS ===
	void publish_goto_setpoint(float x, float y, float z);
	void publish_gps_to_neighbors();
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z, float yaw_rad);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publish_gps_to_neighbors(NeighborsInfo msg);
	void assemble_mode(float min_altitude);
};

void UAVController::timer_callback()
{

	if (offboard_setpoint_counter_ == 10)
	{
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		// Arm the vehicle
		this->arm();
		
	}

	// stop the counter after reaching 11
	if (offboard_setpoint_counter_ < 11)
	{
		// Change to Offboard mode after 10 setpoints

		this->publish_trajectory_setpoint(0.0f, 0.0f, -4.0f, 0.0f);
		this->publish_offboard_control_mode();
		offboard_setpoint_counter_++;

	}
}

// === SUBSCRIBER CALLBACKS ===
// Here is will be action
void UAVController::gps_callback(const VehicleGlobalPosition::SharedPtr msg)
{
	vehicle_gps_position_ = *msg;
}

void UAVController::local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
	vehicle_local_position_ = *msg;
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

void UAVController::publish_goto_setpoint(float x, float y, float z)
{
	GotoSetpoint msg{};
	msg.position = {x, y, z};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
}

void UAVController::publish_offboard_control_mode()
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

void UAVController::publish_trajectory_setpoint(float x, float y, float z, float yaw_rad)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = yaw_rad; // [-PI:PI]
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