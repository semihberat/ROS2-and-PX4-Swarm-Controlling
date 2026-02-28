#include "autonomus.hpp"

// Lifecycle-managed autonomous path planner for swarm member drones

SwarmMemberPathPlanner::SwarmMemberPathPlanner() : LifecycleNode("swarm_member_path_planner")
{
    this->declare_parameter("sys_id", 1);
    this->all_positions.reserve(20);
    this->positioned_drones_.reserve(20);

    // will be in lifecycle
    //  it's currently template to testing
    this->waypoints_ = std::make_shared<Waypoints>();
    this->waypoints_->waypoints = std::vector<VehicleGlobalPosition>(5);
    this->waypoints_->waypoints = {
        VehicleGlobalPosition().set__lat(47.397986).set__lon(8.546056).set__alt(-10.0)};

    this->current_waypoint_ = this->waypoints_->waypoints[0];

    this->in_target_service_ = this->create_service<InTarget>(
        "/in_position",
        std::bind(&SwarmMemberPathPlanner::in_target_callback, this, _1, _2));
        
    this->in_target_client_ = this->create_client<InTarget>("/in_position");
}
