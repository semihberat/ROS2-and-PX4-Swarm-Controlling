#include "autonomous.hpp"

// Store latest neighbor information for path planning
void SwarmMemberPathPlanner::neighbors_info_subscriber(const NeighborsInfo::SharedPtr msg)
{
    this->current_neighbors_info_ = msg;
    current_altitude = msg->main_position.alt;
}

void SwarmMemberPathPlanner::vehicle_attitude_subscriber(const VehicleAttitude::SharedPtr msg)
{
    this->current_euler_angles_ = spatial::ToEulerAngles(msg->q);
}

void SwarmMemberPathPlanner::qr_callback(const QRInformation::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    this->latest_qr_info_ = msg;
    RCLCPP_INFO(this->get_logger(), "\033[1;36mNew QR Info Received! QR ID: %d\033[0m", msg->qr_id);
}