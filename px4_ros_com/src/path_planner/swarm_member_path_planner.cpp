// Standard Libraries
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <tuple>

#include <px4_msgs/msg/vehicle_global_position.hpp>

// Custom Interface for Neighbors Info
#include <custom_interfaces/msg/neighbors_info.hpp>
#include <custom_interfaces/msg/target_positions.hpp>

// Path Planning Formulations
#include "../formulations/geographic/calculate_center_of_gravity.hpp"
#include "../formulations/geographic/calculate_offset_from_center.hpp"
#include "../formulations/geographic/match_drone_with_offset.hpp"
#include "../formulations/geographic/calculate_distance.hpp"

using px4_msgs::msg::VehicleGlobalPosition;
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std::chrono;

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
            std::bind(&SwarmMemberPathPlanner::path_planner_callback, this, _1));
        target_position_publisher_ = this->create_publisher<custom_interfaces::msg::TargetPositions>(tptc, 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&SwarmMemberPathPlanner::timer_callback, this));
    }

private:
    // Subscribers
    rclcpp::Subscription<custom_interfaces::msg::NeighborsInfo>::SharedPtr neighbors_info_subscription_;

    // Publishers
    rclcpp::Publisher<custom_interfaces::msg::TargetPositions>::SharedPtr target_position_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Temporary variables
    double dlat, dlon;
    double target_dlat, target_dlon;
    unsigned int verification_count = 0;
    const unsigned int verification_count_max = 20;

    double offset_lat = 5.0f;
    double offset_lon = 5.0f;

    const unsigned int collision_tolerance_m = 0.5;

    std::vector<px4_msgs::msg::VehicleGlobalPosition> all_positions;

    void path_planner_callback(const custom_interfaces::msg::NeighborsInfo::SharedPtr msg);
    void target_position_publisher();

    void timer_callback()
    {
        target_position_publisher();
    }
};

void SwarmMemberPathPlanner::target_position_publisher()
{
    custom_interfaces::msg::TargetPositions msg{};
    msg.target_dlat = target_dlat;
    msg.target_dlon = target_dlon;
    target_position_publisher_->publish(msg);
};

void SwarmMemberPathPlanner::path_planner_callback(const custom_interfaces::msg::NeighborsInfo::SharedPtr msg)
{

    if (verification_count < verification_count_max)
    {
        verification_count++;
        all_positions = msg->neighbor_positions;
        all_positions.push_back(msg->main_position);
        // rclcpp info all_positions size
        RCLCPP_INFO(this->get_logger(), "All positions size: %zu", all_positions.size());
        auto center_of_gravity = CalculateCenterofGravity().calculate_cog(all_positions);
        auto offsets = CalculateOffsetsFromCenter().calculate_offsets(center_of_gravity, offset_lat, offset_lon, all_positions.size());
        auto matched_position = offsets[msg->main_id - 1];

        std::tie(dlat, dlon, std::ignore) = CalculateDistance().calculate_distance(
            msg->main_position.lat, msg->main_position.lon,
            matched_position.lat, matched_position.lon);
    }
    
    for (auto neighbor : msg->neighbor_positions)
    {
        double x, y, d;
        std::tie(x, y, d) = CalculateDistance().calculate_distance(
            msg->main_position.lat, msg->main_position.lon,
            neighbor.lat, neighbor.lon);
        if (d <= collision_tolerance_m)
        {
            target_dlon = -x;
            target_dlat = -y;
        }
    }

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