#include "communication.hpp"

/** @brief Aggregate and publish neighbor positions to swarm member */
void NeighborsListener::publish_neighbors_info()
{
    NeighborsInfo msg{};
    msg.main_id = this->sys_id;
    msg.main_position = *(this->vehicle_gps_position_);

    msg.neighbor_positions.assign(this->neighbors_gps_queue_.begin(), this->neighbors_gps_queue_.end());
    msg.neighbor_ids = this->neighbors_id_queue_;

    this->neighbors_info_publisher_->publish(msg);
}
