#include "autonomus.hpp"

// Execute synchronized takeoff in formation
void SwarmMemberPathPlanner::formational_takeoff()
{
    double z_error = current_altitude + current_waypoint_.alt;
    double z_vel = std::clamp<float>(z_error * 0.5, -desired_z_vel, desired_z_vel);
    if (std::abs(z_error) <= 0.01 && std::abs(z_vel) <= 0.01)
    {
        bool all_neighbors_in_target = current_neighbors_info_ &&
                                       std::all_of(current_neighbors_info_->neighbor_positions.begin(),
                                                   current_neighbors_info_->neighbor_positions.end(),
                                                   [this](const px4_msgs::msg::VehicleGlobalPosition &pos)
                                                   {
                                                       double neighbor_z_error = -pos.alt - current_waypoint_.alt;
                                                       return std::abs(neighbor_z_error) <= 0.01;
                                                   });
        if (all_neighbors_in_target)
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff complete, all neighbors in position. Moving to next mission.");
            publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
            next_step();
            return;
        }
    }
    publish_trajectory_setpoint(0.0, 0.0, z_vel, 0.0);
}

// Rotate formation while maintaining relative positions
void SwarmMemberPathPlanner::formational_rotation()
{
}

// Navigate to target position
void SwarmMemberPathPlanner::goto_position()
{
}

// Execute mission-specific task at target location
void SwarmMemberPathPlanner::do_process()
{
}

// Complete mission and return to safe state
void SwarmMemberPathPlanner::end_task()
{
}

void SwarmMemberPathPlanner::next_step()
{
    switch (current_mission)
    {
    case Mission::FORMATIONAL_TAKEOFF:
        current_mission = Mission::FORMATIONAL_ROTATION;
        break;
    default:
        current_mission = Mission::FORMATIONAL_TAKEOFF;
        break;
    }
}