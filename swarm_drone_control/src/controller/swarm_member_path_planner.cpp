// Standard Libraries
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <tuple>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
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

#include <px4_msgs/msg/trajectory_setpoint.hpp>

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
    // Subscribers
    rclcpp::Subscription<NeighborsInfo>::SharedPtr neighbors_info_subscription_;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Temporary variables
    double target_dlat, target_dlon;
    VectoralDistance vectoral_distance;
    unsigned int verification_count = 0;
    const unsigned int verification_count_max = 20;
    VehicleVerticalPositions target_position_;
    double offset_lat = 2.0f;
    double offset_lon = 2.0f;

    const double collision_tolerance_m = sqrt(pow(offset_lat, 2) + pow(offset_lon, 2)) / 2.0f;

    std::vector<px4_msgs::msg::VehicleGlobalPosition> all_positions;

    void path_planner_callback(const NeighborsInfo::SharedPtr msg);
    void publish_trajectory_setpoint(float x, float y, float z, float yaw_rad);
    void publish_offboard_control_mode();

    void timer_callback()
    {
        publish_trajectory_setpoint(target_dlat, target_dlon, -4.0, 3.14f);
	    this->publish_offboard_control_mode();
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "ON_CONFIGURE");

        // Create lifecycle publisher
        std::string TRAJECTORY_SETPOINT = "/px4_" + std::to_string(this->get_parameter("sys_id").as_int()) + "/fmu/in/trajectory_setpoint";
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(TRAJECTORY_SETPOINT, 10);
        
        std::string OFFBOARD_CONTROL_MODE = "/px4_" + std::to_string(this->get_parameter("sys_id").as_int()) + "/fmu/in/offboard_control_mode";
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(OFFBOARD_CONTROL_MODE, 10);

        // Create timer (cancelled until activation)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&SwarmMemberPathPlanner::timer_callback, this));
        
        timer_->cancel();
        
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "ON_ACTIVATE");
        
        // Activate lifecycle publisher
        offboard_control_mode_publisher_->on_activate();
        trajectory_setpoint_publisher_->on_activate();
        verification_count = 0;
        // Start timer
        timer_->reset();
        
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "ON_DEACTIVATE");
        
        // Stop timer
        timer_->cancel();
        
        // Deactivate lifecycle publisher
        offboard_control_mode_publisher_->on_deactivate();
        trajectory_setpoint_publisher_->on_deactivate();
        
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "ON_CLEANUP");
        
        // Clean up lifecycle-managed resources
        timer_.reset();
        trajectory_setpoint_publisher_.reset();
        offboard_control_mode_publisher_.reset();
        
        rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "ON_SHUTDOWN");
        
        // Clean up resources on shutdown
        if (timer_) {
            timer_->cancel();
            timer_.reset();
        }
        trajectory_setpoint_publisher_.reset();
        offboard_control_mode_publisher_.reset();
        
        rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }
    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_ERROR(this->get_logger(), "ON_ERROR");
        
        // Clean up resources on error
        if (timer_) {
            timer_->cancel();
            timer_.reset();
        }
        trajectory_setpoint_publisher_.reset();
        offboard_control_mode_publisher_.reset();
        
        rclcpp_lifecycle::LifecycleNode::on_error(previous_state);
        return LifecycleCallbackReturn::FAILURE;
    }
};

void SwarmMemberPathPlanner::publish_offboard_control_mode()
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


}
void SwarmMemberPathPlanner::publish_trajectory_setpoint(float x, float y, float z, float yaw_rad)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = yaw_rad; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
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