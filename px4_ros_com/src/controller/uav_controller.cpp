
#include "include/offboard_controller.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <custom_interfaces/msg/neighbors_info.hpp>

// Mathematical Libraries
#include <cmath>
#include <eigen3/Eigen/Dense>

// isimlendirme yaparken basina / koymak ros2'de namespace kullaniminda daha kati olmasi icin onlem aliyor
// eger / koymazsaniz node ismi namespace'in basina ekleniyor ve bu bazen istenmeyen durumlara yol acabiliyor
// ornegin node ismi offboard_control iken namespace /my_ns ise node ismi /my_ns/offboard_control oluyor

using namespace std::placeholders;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class UAVController : public OffboardController
{
public:
	UAVController() : OffboardController()
	{
		// Listener for GPS and Local Position
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		//Topic Names
		std::string gpstpc = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_global_position";
		std::string lpstpc = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_local_position_v1";
		std::string ntpc = "/px4_" + std::to_string(sys_id) + "/neighbors_info";

		// Subscribers
		vehicle_gps_subscriptions_ = this->create_subscription<VehicleGlobalPosition>(gpstpc, qos,
																					  std::bind(&UAVController::gps_callback, this, _1));
		local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(lpstpc, qos,
																					   std::bind(&UAVController::local_position_callback, this, _1));
		// Listen to neighbor GPS topics
		listen_neighbors(qos); // It's important because we are going to use this function swarm communicating development
		
		// Publishers
		//It comes from controllers/offboard_controller.hpp
		//I am keeping publisher into controllers/offboard_controller.hpp
		neighbors_gps_publisher_ = this->create_publisher<custom_interfaces::msg::NeighborsInfo>(ntpc, 10);
		
		// Timer for publisher callback
		timer_ = this->create_wall_timer(100ms, std::bind(&UAVController::publisher_callback, this));
	}

	VehicleLocalPosition vehicle_local_position_;
	VehicleGlobalPosition vehicle_gps_position_;
	// Here is main function where the publisher and subscriber nodes are created and initialized.
private:
	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_gps_subscriptions_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
	std::vector<VehicleGlobalPosition> neighbor_gps_queue_;
	std::vector<uint8_t> neighbor_id_queue_;
	std::vector<rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr> neighbor_subscriptions_;
	
	float min_altitude = -4.0f;

	// === Main Timer Callback ===

	void publisher_callback()
	{
		if (offboard_setpoint_counter_ == 10)
		{
			// Change to Offboard mode after 10 setpoints
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			// Arm the vehicle
			this->arm();
		}

		// offboard_control_mode needs to be paired with trajectory_setpoint
		publish_offboard_control_mode();
		// When altitude boundary is crossed
		if (vehicle_local_position_.z > min_altitude)
		{
			publish_trajectory_setpoint(0.0, 0.0, -5.0, 3.14);	
		}
		else
		{
			publish_gps_to_neighbors();
			publish_trajectory_setpoint(5.0, 5.0, -5.0, 3.14); // hover at 5 meters
			
		}

		// stop the counter after reaching 11
		if (offboard_setpoint_counter_ < 11)
		{
			offboard_setpoint_counter_++;
		}
	}

	// === SUBSCRIBER CALLBACKS ===
	void gps_callback(const VehicleGlobalPosition::SharedPtr msg);
	void local_position_callback(const VehicleLocalPosition::SharedPtr msg);

	// Listen Neighbors 
	void listen_neighbors(rclcpp::QoS qos);
	void neighbor_gps_callback(const VehicleGlobalPosition::SharedPtr msg);

	// === PUBLISH FUNCTIONS ===
	void publish_gps_to_neighbors();
};

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

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UAVController>());
	rclcpp::shutdown();
	return 0;
}

// Calculate central wei