#include "communication.hpp"

/** @brief Constructor - setup parameters and subscriptions */
NeighborsListener::NeighborsListener() : Node("neighbors_listener")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    this->declare_parameter("sys_id", 1);
    this->declare_parameter("total_drones", 1);
    this->sys_id = this->get_parameter("sys_id").as_int();
    this->total_drones = this->get_parameter("total_drones").as_int();
    std::string vehicle_global_position_topic = "/px4_" + std::to_string(this->sys_id) + "/fmu/out/vehicle_global_position";
    std::string vehicle_attitude_topic = "/px4_" + std::to_string(this->sys_id) + "/fmu/out/vehicle_attitude";
    std::string neighbors_info_topic = "/px4_" + std::to_string(this->sys_id) + "/neighbors_info";
    std::string vehicle_local_position_topic = "/px4_" + std::to_string(this->sys_id) + "/fmu/out/vehicle_local_position_v1";

    this->neighbors_info_publisher_ = this->create_publisher<NeighborsInfo>(neighbors_info_topic, 10);

    this->vehicle_gps_subscription_ = this->create_subscription<VehicleGlobalPosition>(vehicle_global_position_topic, qos,
                                                                                       std::bind(&NeighborsListener::gps_callback, this, _1));
    this->vehicle_attitude_subscription_ = this->create_subscription<VehicleAttitude>(vehicle_attitude_topic, qos,
                                                                                      std::bind(&NeighborsListener::attitude_callback, this, _1));
    this->local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(vehicle_local_position_topic, qos,
                                                                                         std::bind(&NeighborsListener::local_position_callback, this, _1));

    this->setup_neighbor_listeners(qos);

    this->timer_ = this->create_wall_timer(
        100ms, std::bind(&NeighborsListener::timer_callback, this));
}

/** @brief Subscribe to all neighbor drone GPS topics */
void NeighborsListener::setup_neighbor_listeners(const rclcpp::QoS &qos)
{
    this->neighbor_subscriptions_.reserve(this->total_drones - 1);
    this->neighbor_att_subscriptions_.reserve(this->total_drones - 1);
    this->neighbors_id_queue_.reserve(this->total_drones - 1);
    this->neighbors_yaw_queue_.resize(this->total_drones - 1, 0.0f);

    for (int i = 1; i <= this->total_drones; ++i)
    {
        if (i != this->sys_id)
        {
            std::string n_gps_topic = "/px4_" + std::to_string(i) + "/fmu/out/vehicle_global_position";
            auto n_gps_sub = this->create_subscription<VehicleGlobalPosition>(n_gps_topic, qos, 
                std::bind(&NeighborsListener::neighbor_gps_callback, this, _1));
            this->neighbor_subscriptions_.push_back(n_gps_sub);

            std::string n_att_topic = "/px4_" + std::to_string(i) + "/fmu/out/vehicle_attitude";
            auto n_att_sub = this->create_subscription<VehicleAttitude>(n_att_topic, qos,
                [this, i](const VehicleAttitude::SharedPtr msg) { this->neighbor_attitude_callback(msg, i); });
            this->neighbor_att_subscriptions_.push_back(n_att_sub);

            this->neighbors_id_queue_.push_back(i);
        }
    }
}
