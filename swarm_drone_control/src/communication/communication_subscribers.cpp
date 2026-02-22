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
