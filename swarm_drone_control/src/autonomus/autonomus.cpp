#include "autonomus.hpp"

// Lifecycle-managed autonomous path planner for swarm member drones
SwarmMemberPathPlanner::SwarmMemberPathPlanner() : LifecycleNode("swarm_member_path_planner")
{
    this->declare_parameter("sys_id", 1);
    this->all_positions.reserve(20);
}