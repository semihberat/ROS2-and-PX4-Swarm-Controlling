#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <px4_msgs/msg/vehicle_global_position.hpp>

// Custom Interface for Neighbors Info
#include <custom_interfaces/msg/neighbors_info.hpp>
#include <custom_interfaces/msg/target_positions.hpp>

#include "../formulations/geographic/calculate_center_of_gravity.hpp"
#include "../formulations/geographic/calculate_offset_from_center.hpp"
#include "../formulations/geographic/match_drone_with_offset.hpp"
#include "../formulations/geographic/calculate_distance.hpp"
#include <tuple>

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
        std::string tptc = "/px4_" + std::to_string(this->get_parameter("sys_id").as_int()) + "/target_positions";

        neighbors_info_subscription_ = this->create_subscription<custom_interfaces::msg::NeighborsInfo>(
            ntpc, qos,
            std::bind(&SwarmMemberPathPlanner::path_planner_callback, this, _1)
        );
        target_position_publisher_ = this->create_publisher<custom_interfaces::msg::TargetPositions>(tptc, 10);
    }   

    std::vector<VehicleGlobalPosition> all_positions;
    double target_dlat, target_dlon;
    
private:
    rclcpp::Subscription<custom_interfaces::msg::NeighborsInfo>::SharedPtr neighbors_info_subscription_;

    rclcpp::Publisher<custom_interfaces::msg::TargetPositions>::SharedPtr target_position_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    
    void path_planner_callback(const custom_interfaces::msg::NeighborsInfo::SharedPtr msg);
    void target_position_publisher();

    void timer_callback(){
        target_position_publisher();
    }
};

void SwarmMemberPathPlanner::target_position_publisher(){
        custom_interfaces::msg::TargetPositions msg{};
        msg.target_dlat = target_dlat;
        msg.target_dlon = target_dlon;
        target_position_publisher_->publish(msg);
    };

void SwarmMemberPathPlanner::path_planner_callback(const custom_interfaces::msg::NeighborsInfo::SharedPtr msg){
        all_positions = msg->neighbor_positions;
        all_positions.push_back(msg->main_position);
        auto center_of_gravity = CalculateCenterofGravity().calculate_cog(all_positions);
        auto offsets = CalculateOffsetsFromCenter().calculate_offsets(center_of_gravity, 5.0f, 5.0f, all_positions.size());
        auto matched_position = MatchDroneWithOffset().match(msg->main_position, offsets); 
        double dlat, dlon;
        std::tie(dlat, dlon, std::ignore) = CalculateDistance().calculate_distance(
            msg->main_position.lat, msg->main_position.lon,
            matched_position.lat, matched_position.lon
        );
        target_dlat = dlat;
        target_dlon = dlon;

    }

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SwarmMemberPathPlanner>());
	rclcpp::shutdown();
	return 0;
}
// 31 10 2025