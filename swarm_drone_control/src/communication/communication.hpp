#ifndef SWARM_COMMUNICATION_HPP
#define SWARM_COMMUNICATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <custom_interfaces/msg/neighbors_info.hpp>

using namespace std::placeholders;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace custom_interfaces::msg;

/**
 * @brief Aggregates neighbor drone positions and publishes to swarm members
 */
class NeighborsListener : public rclcpp::Node
{
public:
    NeighborsListener();

private:
    // Publishers and Subscriber callbacks
    rclcpp::Publisher<NeighborsInfo>::SharedPtr neighbors_info_publisher_;
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_gps_subscription_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;

    /** @brief Timer callback for periodic updates */
    void timer_callback();

    /** @brief Aggregate and publish neighbor positions */
    void publish_neighbors_info();

    /** @brief Handle own GPS position */
    void gps_callback(const VehicleGlobalPosition::SharedPtr msg);

    /** @brief Handle own local position */
    void local_position_callback(const VehicleLocalPosition::SharedPtr msg);

    /** @brief Handle own attitude (yaw) */
    void attitude_callback(const VehicleAttitude::SharedPtr msg);

    /** @brief Subscribe to all neighbor GPS topics */
    void setup_neighbor_listeners(const rclcpp::QoS &qos);

    /** @brief Collect neighbor GPS data */
    void neighbor_gps_callback(const VehicleGlobalPosition::SharedPtr msg);

    /** @brief Collect neighbor attitude data */
    void neighbor_attitude_callback(const VehicleAttitude::SharedPtr msg, int neighbor_idx);

    // Parameters
    uint8_t sys_id;
    uint8_t total_drones;
    VehicleGlobalPosition::SharedPtr vehicle_gps_position_ = std::make_shared<VehicleGlobalPosition>();
    VehicleLocalPosition::SharedPtr vehicle_local_position_ = std::make_shared<VehicleLocalPosition>();
    float main_yaw_ = 0.0;

    rclcpp::TimerBase::SharedPtr timer_;

    // Data Queues
    std::vector<rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr> neighbor_subscriptions_;
    std::vector<rclcpp::Subscription<VehicleAttitude>::SharedPtr> neighbor_att_subscriptions_;
    std::deque<VehicleGlobalPosition> neighbors_gps_queue_;
    std::vector<float> neighbors_yaw_queue_;
    std::vector<uint8_t> neighbors_id_queue_;
};

#endif // SWARM_COMMUNICATION_HPP