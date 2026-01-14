#ifndef SWARM_COMMUNICATION_HPP
#define SWARM_COMMUNICATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <deque>

// PX4_MSGS SUBSCRIPTIONS
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/goto_setpoint.hpp>

// CUSTOM INTERFACES
#include <custom_interfaces/msg/neighbors_info.hpp>
#include <custom_interfaces/msg/target_positions.hpp>

using namespace std::placeholders;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace custom_interfaces::msg;

class NeighborsListener : public rclcpp::Node
{
public:
    NeighborsListener();

private:
    // Timer callback function
    void timer_callback();

    // Parameters
    uint8_t sys_id;
    uint8_t number_of_drones;
    VehicleGlobalPosition::SharedPtr vehicle_gps_position_ = std::make_shared<VehicleGlobalPosition>();
    VehicleLocalPosition::SharedPtr vehicle_local_position_ = std::make_shared<VehicleLocalPosition>();

    rclcpp::TimerBase::SharedPtr timer_;

    // Data Queues
    std::deque<VehicleGlobalPosition> neighbor_gps_queue_;
    std::vector<uint8_t> neighbor_id_queue_;

    rclcpp::Subscription<TargetPositions>::SharedPtr target_position_subscription_;
    rclcpp::Publisher<NeighborsInfo>::SharedPtr neighbors_gps_publisher_;
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_gps_subscriptions_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;

    std::vector<rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr> neighbor_subscriptions_;

    // Callback functions
    void publish_gps_to_neighbors();
    void gps_callback(const VehicleGlobalPosition::SharedPtr msg);
    void local_position_callback(const VehicleLocalPosition::SharedPtr msg);
    void listen_neighbors(const rclcpp::QoS &qos);
    void neighbor_gps_callback(const VehicleGlobalPosition::SharedPtr msg);
};

#endif // SWARM_COMMUNICATION_HPP
