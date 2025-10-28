#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <px4_msgs/msg/vehicle_global_position.hpp>

// Custom Interface for Neighbors Info
#include <custom_interfaces/msg/neighbors_info.hpp>

#include "../formulations/calculate_center_of_gravity.hpp"
#include "../formulations/calculate_offset_from_center.hpp"

using std::placeholders::_1;
using px4_msgs::msg::VehicleGlobalPosition;

class SwarmMemberPathPlanner : public rclcpp::Node
{
public:
    SwarmMemberPathPlanner() : Node("swarm_member_path_planner")
    {
        // Initialize QoS profile for sensor data
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Declare parameter for each swarm member's system ID
        this->declare_parameter("sys_id", 1);

        std::string ntpc = "/px4_" + std::to_string(this->get_parameter("sys_id").as_int()) + "/neighbors_info";

        neighbors_info_subscription_ = this->create_subscription<custom_interfaces::msg::NeighborsInfo>(
            ntpc, qos,
            std::bind(&SwarmMemberPathPlanner::path_planner_callback, this, _1)
        );
    }   
    
private:
    rclcpp::Subscription<custom_interfaces::msg::NeighborsInfo>::SharedPtr neighbors_info_subscription_;

    void path_planner_callback(const custom_interfaces::msg::NeighborsInfo::SharedPtr msg){
        auto center_of_gravity = CalculateCenterofGravity().calculate_cog(msg->neighbor_positions);
        auto offsets = CalculateOffsetsFromCenter().calculate_offsets(center_of_gravity, 5.0f, 5.0f, msg->neighbor_positions.size());
        
    }
};

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SwarmMemberPathPlanner>());
	rclcpp::shutdown();
	return 0;
}