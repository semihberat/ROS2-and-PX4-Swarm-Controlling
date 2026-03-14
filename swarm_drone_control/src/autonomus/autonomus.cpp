#include "autonomus.hpp"

// Lifecycle-managed autonomous path planner for swarm member drones

SwarmMemberPathPlanner::SwarmMemberPathPlanner() : LifecycleNode("swarm_member_path_planner")
{
    this->declare_parameter("sys_id", 1);
    this->all_positions.reserve(20);
    this->positioned_drones_.reserve(20);

    // will be in lifecycle
    //  it's currently template to testing
    std::vector<VehicleGlobalPosition> test_waypoints;

    // Waypoint 1 (Başlangıç Hedefi)
    test_waypoints.push_back(VehicleGlobalPosition().set__lat(47.397986).set__lon(8.546056).set__alt(target_altitude_));

    // Waypoint 2 (Kuzeye doğru ~20 metre)
    test_waypoints.push_back(VehicleGlobalPosition().set__lat(47.398186).set__lon(8.546056).set__alt(target_altitude_));

    // Waypoint 3 (Doğuya doğru ~20 metre)
    test_waypoints.push_back(VehicleGlobalPosition().set__lat(47.398186).set__lon(8.546256).set__alt(target_altitude_));

    this->waypoint_manager_.set_waypoints(test_waypoints);

    this->in_target_service_ = this->create_service<InTarget>(
        "/in_position",
        std::bind(&SwarmMemberPathPlanner::in_target_callback, this, _1, _2));

    this->in_target_client_ = this->create_client<InTarget>("/in_position");
}
