#include "autonomus.hpp"

// Lifecycle-managed autonomous path planner for swarm member drones
SwarmMemberPathPlanner::SwarmMemberPathPlanner() : LifecycleNode("swarm_member_path_planner")
{
    this->declare_parameter("sys_id", 1);
    this->all_positions.reserve(20);

    // will be in lifecycle
    //  it's currently template to testing
    this->waypoints_ = std::make_shared<Waypoints>();
    this->waypoints_->waypoints = std::vector<VehicleGlobalPosition>(5);
    this->waypoints_->waypoints = {
        VehicleGlobalPosition().set__lat(0.0).set__lon(0.0).set__alt(-10.0)
    };

    this->current_waypoint_ = this->waypoints_->waypoints[0];
}