#include "communication.hpp"

/** @brief Update own GPS position */
void NeighborsListener::gps_callback(const VehicleGlobalPosition::SharedPtr msg)
{
    this->vehicle_gps_position_ = msg;
}

/** @brief Update own local position */
void NeighborsListener::local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
    this->vehicle_local_position_ = msg;
}

/** @brief Collect neighbor GPS data in FIFO queue */
void NeighborsListener::neighbor_gps_callback(const VehicleGlobalPosition::SharedPtr msg)
{
    if (this->neighbors_gps_queue_.size() >= static_cast<size_t>(this->total_drones - 1))
    {
        this->neighbors_gps_queue_.pop_front();
    }
    this->neighbors_gps_queue_.push_back(*msg);
}
void NeighborsListener::attitude_callback(const VehicleAttitude::SharedPtr msg)
{
    this->main_yaw_ = atan2f(2.0f * (msg->q[0] * msg->q[3] + msg->q[1] * msg->q[2]),
                             1.0f - 2.0f * (msg->q[2] * msg->q[2] + msg->q[3] * msg->q[3]));
}

void NeighborsListener::neighbor_attitude_callback(const VehicleAttitude::SharedPtr msg, int neighbor_idx)
{
    float neighbor_yaw = atan2f(2.0f * (msg->q[0] * msg->q[3] + msg->q[1] * msg->q[2]),
                                1.0f - 2.0f * (msg->q[2] * msg->q[2] + msg->q[3] * msg->q[3]));
    
    // Find index in neighbors_id_queue_
    for (size_t i = 0; i < this->neighbors_id_queue_.size(); ++i) {
        if (this->neighbors_id_queue_[i] == neighbor_idx) {
            if (i < this->neighbors_yaw_queue_.size()) {
                this->neighbors_yaw_queue_[i] = neighbor_yaw;
            }
            break;
        }
    }
}

