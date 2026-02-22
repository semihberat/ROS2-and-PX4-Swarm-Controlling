#include "autonomus.hpp"

// Store latest neighbor information for path planning
void SwarmMemberPathPlanner::neighbors_info_subscriber(const NeighborsInfo::SharedPtr msg)
{
    this->current_neighbors_info_ = msg;
    current_altitude = msg->main_position.alt;
}