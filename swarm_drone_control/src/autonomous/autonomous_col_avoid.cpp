#include "autonomous.hpp"

// Apply computed collision avoidance bias to final target velocities
void SwarmMemberPathPlanner::apply_collision_bias()
{
    current_commands.v_lat += collision_bias.vlat;
    current_commands.v_lon += collision_bias.vlon;
}

void SwarmMemberPathPlanner::collision_avoidance()
{
}
