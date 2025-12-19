// Standard Libraries
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <tuple>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Custom Interface for Neighbors Info
#include <custom_interfaces/msg/neighbors_info.hpp>
#include <custom_interfaces/msg/target_positions.hpp>

// Path Planning Formulations
#include "../formulations/geographic/calculate_center_of_gravity.hpp"
#include "../formulations/geographic/calculate_offset_from_center.hpp"
#include "../formulations/geographic/calculate_distance.hpp"

// interfaces
#include "../interfaces/vehicle_positions.hpp"
#include "../interfaces/vectoral_distance.hpp"

#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace lifecycle_msgs::srv;
using namespace lifecycle_msgs::msg;
using namespace px4_msgs::msg;
using namespace custom_interfaces::msg;
using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace std::chrono;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SwarmMemberPathPlanner : public rclcpp_lifecycle::LifecycleNode
{
public:
    SwarmMemberPathPlanner() : LifecycleNode("swarm_member_path_planner")
    {
        // Initialize QoS profile for sensor data
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Declare parameter for each swarm member's system ID
        this->declare_parameter("sys_id", 1);

        std::string ntpc = "/px4_" + std::to_string(this->get_parameter("sys_id").as_int()) + "/neighbors_info";

        neighbors_info_subscription_ = this->create_subscription<NeighborsInfo>(
            ntpc, qos,
            std::bind(&SwarmMemberPathPlanner::path_planner_callback, this, _1));
    }

private:
    // Service
    rclcpp::Service<GetState>::SharedPtr get_state_service_;
    rclcpp::Service<GetAvailableStates>::SharedPtr get_available_states_service_;
    // Client
    rclcpp::Client<ChangeState>::SharedPtr change_state_client_;

    // Subscribers
    rclcpp::Subscription<NeighborsInfo>::SharedPtr neighbors_info_subscription_;

    // Publishers
    rclcpp::Publisher<TargetPositions>::SharedPtr target_position_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Temporary variables
    double target_dlat, target_dlon;
    VectoralDistance vectoral_distance;
    unsigned int verification_count = 0;
    const unsigned int verification_count_max = 50;
    VehicleVerticalPositions target_position_;
    double offset_lat = 2.0f;
    double offset_lon = 2.0f;

    const double collision_tolerance_m = sqrt(pow(offset_lat, 2) + pow(offset_lon, 2)) / 2.0f;

    std::vector<px4_msgs::msg::VehicleGlobalPosition> all_positions;

    void path_planner_callback(const NeighborsInfo::SharedPtr msg);
    
    void target_position_publisher();

    void timer_callback()
    {
        target_position_publisher();
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "ON_CONFIGURE");
        std::string tptc = "/px4_" + std::to_string(this->get_parameter("sys_id").as_int()) + "/target_positions";
        target_position_publisher_ = this->create_publisher<TargetPositions>(tptc, 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&SwarmMemberPathPlanner::timer_callback, this));
        timer_->cancel();
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "ON_ACTIVATE");
        timer_->reset();
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "ON_DEACTIVATE");
        timer_->cancel();
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "ON_CLEANUP");
        timer_.reset();
        target_position_publisher_.reset();
        rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "ON_SHUTDOWN");
        rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }
    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "ON_ERROR");
        rclcpp_lifecycle::LifecycleNode::on_error(previous_state);
        return LifecycleCallbackReturn::FAILURE;
    }
};

void SwarmMemberPathPlanner::target_position_publisher()
{
    TargetPositions msg{};
    msg.target_dlat = target_dlat;
    msg.target_dlon = target_dlon;
    target_position_publisher_->publish(msg);
};

// Path Planner Callback Function
void SwarmMemberPathPlanner::path_planner_callback(const NeighborsInfo::SharedPtr msg)
{

    if (verification_count < verification_count_max)
    {
        verification_count++;
        all_positions = msg->neighbor_positions;
        all_positions.push_back(msg->main_position);

        auto center_of_gravity = CalculateCenterofGravity().calculate_cog<VehicleVerticalPositions, VehicleGlobalPosition>(all_positions);
        auto offsets = CalculateOffsetsFromCenter().calculate_offsets(center_of_gravity, offset_lat, offset_lon, all_positions.size());
        auto matched_position = offsets[msg->main_id - 1];

        target_position_ = matched_position;

        vectoral_distance = CalculateDistance().calculate_distance<VectoralDistance>(
            msg->main_position.lat, msg->main_position.lon,
            matched_position.lat, matched_position.lon);
    }

    target_dlat = vectoral_distance.dlat_meter;
    target_dlon = vectoral_distance.dlon_meter;

    for (const auto &neighbor : msg->neighbor_positions)
    {
        auto uav_distance = CalculateDistance().calculate_distance<VectoralDistance>(
            msg->main_position.lat, msg->main_position.lon,
            neighbor.lat, neighbor.lon);

        if (uav_distance.distance <= collision_tolerance_m)
        {
            target_dlon = -uav_distance.dlon_meter + target_dlon;
            target_dlat = -uav_distance.dlat_meter + target_dlat;
        }
    }

    float distance = sqrt(pow(target_dlat, 2) + pow(target_dlon, 2));

    if ((distance <= 0.01f) && (verification_count >= verification_count_max))
    {
        if(this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE){
        this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
    }
}

int main(int argc, char *argv[])
{
    std::cout << "============== Path Planer ==============" << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmMemberPathPlanner>()->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
// 31 10 2025