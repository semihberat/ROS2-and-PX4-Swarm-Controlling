#include <rclcpp/rclcpp.hpp>

// ========== PX4_MSGS ==========

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
    NeighborsListener() : Node("neighbors_listener")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        this->declare_parameter("sys_id", 1);
        this->declare_parameter("number_of_drones", 1);
        sys_id = this->get_parameter("sys_id").as_int();
        number_of_drones = this->get_parameter("number_of_drones").as_int();

        std::string vehicle_global_position_topic = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_global_position";
        std::string neighbors_info_topic = "/px4_" + std::to_string(sys_id) + "/neighbors_info";
        std::string vehicle_local_position_topic = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_local_position_v1";

        neighbors_gps_publisher_ = this->create_publisher<NeighborsInfo>(neighbors_info_topic, 10);

        // Subscription topic namespace
        vehicle_gps_subscriptions_ = this->create_subscription<VehicleGlobalPosition>(vehicle_global_position_topic, qos,
                                                                                      std::bind(&NeighborsListener::gps_callback, this, _1));

        local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(vehicle_local_position_topic, qos,
                                                                                       std::bind(&NeighborsListener::local_position_callback, this, _1));

        listen_neighbors(qos);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&NeighborsListener::publish_gps_to_neighbors, this));
    }

private:
    // Parameters
    uint8_t sys_id;
    uint8_t number_of_drones;
    VehicleGlobalPosition vehicle_gps_position_;
    VehicleLocalPosition vehicle_local_position_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data Queues
    std::vector<VehicleGlobalPosition> neighbor_gps_queue_;
    std::vector<uint8_t> neighbor_id_queue_;

    rclcpp::Subscription<TargetPositions>::SharedPtr target_position_subscription_;
    rclcpp::Publisher<NeighborsInfo>::SharedPtr neighbors_gps_publisher_;
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_gps_subscriptions_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;

    std::vector<rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr> neighbor_subscriptions_;

    void publish_gps_to_neighbors();
    void gps_callback(const VehicleGlobalPosition::SharedPtr msg);
    void local_position_callback(const VehicleLocalPosition::SharedPtr msg);
    void listen_neighbors(rclcpp::QoS qos);
    void neighbor_gps_callback(const VehicleGlobalPosition::SharedPtr msg);

    void timer_callback();
};

void NeighborsListener::timer_callback()
{
    publish_gps_to_neighbors();
}

void NeighborsListener::gps_callback(const VehicleGlobalPosition::SharedPtr msg)
{
    vehicle_gps_position_ = *msg;
}

void NeighborsListener::local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
    vehicle_local_position_ = *msg;
}

void NeighborsListener::listen_neighbors(rclcpp::QoS qos)
{
    for (uint8_t i = 1; i <= number_of_drones; i++)
    {
        if (i != sys_id)
        {
            std::string member_topic = "/px4_" + std::to_string(i) + "/fmu/out/vehicle_global_position";
            neighbor_id_queue_.push_back(i);
            auto sub = this->create_subscription<VehicleGlobalPosition>(member_topic, qos,
                                                                        std::bind(&NeighborsListener::neighbor_gps_callback, this, _1));

            neighbor_subscriptions_.push_back(sub);
        }
    }
}

void NeighborsListener::publish_gps_to_neighbors()
{
    NeighborsInfo msg{};
    msg.main_id = sys_id;
    msg.main_position = vehicle_gps_position_;
    msg.neighbor_positions = neighbor_gps_queue_;
    msg.neighbor_ids = neighbor_id_queue_;

    neighbors_gps_publisher_->publish(msg);
}

void NeighborsListener::neighbor_gps_callback(const VehicleGlobalPosition::SharedPtr msg)
{
    if (neighbor_gps_queue_.size() >= static_cast<size_t>(number_of_drones - 1))
    {
        neighbor_gps_queue_.erase(neighbor_gps_queue_.begin());
    }
    neighbor_gps_queue_.push_back(*msg);
}

int main(int argc, char *argv[])
{
    std::cout << "============== UAV Controller ==============" << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NeighborsListener>());
    rclcpp::shutdown();
    return 0;
}