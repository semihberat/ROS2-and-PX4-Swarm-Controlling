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

//CUSTOM INTERFACES
#include <custom_interfaces/msg/neighbors_info.hpp>
#include <custom_interfaces/msg/target_positions.hpp>

// isimlendirme yaparken basina / koymak ros2'de namespace kullaniminda daha kati olmasi icin onlem aliyor
// eger / koymazsaniz node ismi namespace'in basina ekleniyor ve bu bazen istenmeyen durumlara yol acabiliyor
// ornegin node ismi offboard_control iken namespace /my_ns ise node ismi /my_ns/offboard_control oluyor

using namespace std::placeholders;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class UAVController : public rclcpp::Node
{
public:
	UAVController() : rclcpp::Node("uav_controller")
	{
		this->declare_parameter("sys_id", 1);
		this->declare_parameter("number_of_drones", 1);
		sys_id = this->get_parameter("sys_id").as_int();
		number_of_drones = this->get_parameter("number_of_drones").as_int();

		// Listener for GPS and Local Position
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		//Topic Names
		std::string ocmptpc = "/px4_" + std::to_string(sys_id) + "/fmu/in/offboard_control_mode";
		std::string tsptpc = "/px4_" + std::to_string(sys_id) + "/fmu/in/trajectory_setpoint";
		std::string vctpc = "/px4_" + std::to_string(sys_id) + "/fmu/in/vehicle_command";
		std::string gpstpc = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_global_position";
		std::string lpstpc = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_local_position_v1";
		std::string ntpc = "/px4_" + std::to_string(sys_id) + "/neighbors_info"; 
		std::string tpc = "/px4_" + std::to_string(sys_id) + "/target_positions";

		// Subscribers
		target_position_subscription_ = this->create_subscription<custom_interfaces::msg::TargetPositions>(tpc, qos,
																											  std::bind(&UAVController::target_position_callback, this, _1));

		vehicle_gps_subscriptions_ = this->create_subscription<VehicleGlobalPosition>(gpstpc, qos,
																					  std::bind(&UAVController::gps_callback, this, _1));
		local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(lpstpc, qos,
																					   std::bind(&UAVController::local_position_callback, this, _1));
		// Listen to neighbor GPS topics
		listen_neighbors(qos); // It's important because we are going to use this function swarm communicating development
		
		// Publishers
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(ocmptpc, 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(tsptpc, 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(vctpc, 10);
		neighbors_gps_publisher_ = this->create_publisher<custom_interfaces::msg::NeighborsInfo>(ntpc, 10);
		
		offboard_setpoint_counter_ = 0;

		// Timer for publisher callback
		timer_ = this->create_wall_timer(100ms, std::bind(&UAVController::publisher_callback, this));
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
	rclcpp::Subscription<custom_interfaces::msg::TargetPositions>::SharedPtr target_position_subscription_;
	std::vector<rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr> neighbor_subscriptions_;

	// Publishers
	rclcpp::Publisher<custom_interfaces::msg::NeighborsInfo>::SharedPtr neighbors_gps_publisher_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	// Data Queues
	std::vector<VehicleGlobalPosition> neighbor_gps_queue_;
	std::vector<uint8_t> neighbor_id_queue_;

	// Parameters
	rclcpp::TimerBase::SharedPtr timer_;
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	uint8_t sys_id;
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	uint8_t number_of_drones;
	double target_dlat = 0.0;
	double target_dlon = 0.0;
	float min_altitude = -4.0f;
	 int count = 0;

	// === Main Timer Callback ===
	void publisher_callback();

	// === SUBSCRIBER CALLBACKS ===
	void gps_callback(const VehicleGlobalPosition::SharedPtr msg);
	void local_position_callback(const VehicleLocalPosition::SharedPtr msg);
	void target_position_callback(const custom_interfaces::msg::TargetPositions::SharedPtr msg);

	// Listen Neighbors 
	void listen_neighbors(rclcpp::QoS qos);
	void neighbor_gps_callback(const VehicleGlobalPosition::SharedPtr msg);

	// === PUBLISH FUNCTIONS ===
	void publish_goto_setpoint(float x, float y, float z);
	void publish_gps_to_neighbors();
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z, float yaw_rad);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publish_gps_to_neighbors(custom_interfaces::msg::NeighborsInfo msg);
};

void UAVController::publisher_callback()
	{
		
		if (offboard_setpoint_counter_ == 10)
		{
			// Change to Offboard mode after 10 setpoints
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			// Arm the vehicle
			this->arm();
		}

		// offboard_control_mode needs to be paired with trajectory_setpoint
		this->publish_offboard_control_mode();
		// When altitude boundary is crossed
		if (vehicle_local_position_.z > min_altitude)
		{
			this->publish_trajectory_setpoint(0.0, 0.0, min_altitude-1, 3.14);
		}
		else
		{
			this->publish_trajectory_setpoint(target_dlat, target_dlon, min_altitude-1, 3.14);
		}

		// stop the counter after reaching 11
		if (offboard_setpoint_counter_ < 11)
		{
			offboard_setpoint_counter_++;
		}
	}

void UAVController::target_position_callback(const custom_interfaces::msg::TargetPositions::SharedPtr msg){
		target_dlat = msg->target_dlat;
		target_dlon = msg->target_dlon;
	}	

void UAVController::gps_callback(const VehicleGlobalPosition::SharedPtr msg){
		vehicle_gps_position_ = *msg;
	}

void UAVController::local_position_callback(const VehicleLocalPosition::SharedPtr msg){
		vehicle_local_position_ = *msg;
	}

void UAVController::listen_neighbors(rclcpp::QoS qos){
		for (uint8_t i = 1; i <= number_of_drones; i++)
		{
			if (i != sys_id)
			{
				std::string member_topic = "/px4_" + std::to_string(i) + "/fmu/out/vehicle_global_position";
				neighbor_id_queue_.push_back(i);
				auto sub = this->create_subscription<VehicleGlobalPosition>(member_topic, qos,
																			std::bind(&UAVController::neighbor_gps_callback, this, _1));

				neighbor_subscriptions_.push_back(sub);
			}
		}
	}

void UAVController::neighbor_gps_callback(const VehicleGlobalPosition::SharedPtr msg)
	{
		if(neighbor_gps_queue_.size() >= static_cast<size_t>(number_of_drones - 1)){
			neighbor_gps_queue_.erase(neighbor_gps_queue_.begin());
		}
		neighbor_gps_queue_.push_back(*msg); 
	}

void UAVController::publish_gps_to_neighbors(){
		custom_interfaces::msg::NeighborsInfo msg{};
		msg.main_id = sys_id;
		msg.main_position = vehicle_gps_position_;
		msg.neighbor_positions = neighbor_gps_queue_;
		msg.neighbor_ids = neighbor_id_queue_;
		
		neighbors_gps_publisher_->publish(msg);
	}

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

void UAVController::publish_goto_setpoint(float x, float y, float z){
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
	msg.velocity = {x, y, z};

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
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UAVController>());
	rclcpp::shutdown();
	return 0;
}