#include "autonomous.hpp"

// Execute synchronized takeoff in formation
void SwarmMemberPathPlanner::formational_takeoff()
{
    double z_error = current_altitude + current_wp_->alt;
    current_commands.z_vel = autonomous_utils::calculate_velocity<double>(z_error, desired_velocities.z_vel);

    if (verify_in_target_state(std::abs(z_error)))
    {
        next_step(Mission::FORMATIONAL_ROTATION);
        return;
    }

    bool local_in_target;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        local_in_target = drones_in_target_[sys_id_];
    }

    if (local_in_target)
    {
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
    }
    else
    {
        publish_trajectory_setpoint(0.0, 0.0, current_commands.z_vel, 0.0);
    }
}

// Rotate formation while maintaining relative positions
void SwarmMemberPathPlanner::formational_rotation()
{
    this->target_distance_ = geo::calculate_distance<DLatDLon>(this->current_neighbors_info_->main_position.lat,
                                                               this->current_neighbors_info_->main_position.lon,
                                                               swarm_positions.circular_position.lat, swarm_positions.circular_position.lon);

    update_velocity();
    current_commands.yaw_vel = autonomous_utils::calculate_velocity<double>(this->target_distance_.dlat_meter, desired_velocities.yaw_vel);

    // Collision avoidance modifying speeds
    apply_collision_bias();

    // Check Neighbors
    if (verify_in_target_state(std::abs(this->target_distance_.distance)))
    {
        next_step(Mission::GOTO_POSITION);
        return;
    }

    bool local_in_target;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        local_in_target = drones_in_target_[sys_id_];
    }

    if (local_in_target)
    {
        // Wait gracefully
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
        return;
    }
    publish_trajectory_setpoint(current_commands.v_lat, current_commands.v_lon, 0.0, current_commands.yaw_vel);
}

// Navigate to target position
void SwarmMemberPathPlanner::goto_position()
{
    this->target_distance_ = geo::calculate_distance<DLatDLon>(this->current_neighbors_info_->main_position.lat,
                                                               this->current_neighbors_info_->main_position.lon,
                                                               swarm_positions.target_after_offset.lat, swarm_positions.target_after_offset.lon);

    update_velocity();
    current_commands.bearing = geo::calculate_bearing<VehicleGlobalPosition>(this->current_neighbors_info_->main_position, swarm_positions.target_after_offset);
    current_commands.yaw_vel = autonomous_utils::calculate_velocity<double>(current_commands.bearing, desired_velocities.yaw_vel);

    // Collision avoidance modifying speeds
    apply_collision_bias();

    if (verify_in_target_state(std::abs(this->target_distance_.distance)))
    {
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0); // Stop at target
        current_wp_ = waypoint_manager_.next();
        next_step(Mission::DO_PROCESS);
        return;
    }

    bool local_in_target;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        local_in_target = drones_in_target_[sys_id_];
    }

    if (local_in_target)
    {
        // Wait gracefully
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
        return;
    }

    publish_trajectory_setpoint(current_commands.v_lat, current_commands.v_lon, 0.0, current_commands.yaw_vel);
}

// Complete mission and return to safe state
void SwarmMemberPathPlanner::end_task()
{
}

void SwarmMemberPathPlanner::next_step(Mission next_mission)
{
    this->current_mission = next_mission;

    // 1. Advance mission state
    switch (this->current_mission)
    {
    case Mission::FORMATIONAL_ROTATION:
        LOG_MISSION(this->get_logger(), "[MISSION INFO] -> FORMATIONAL_ROTATION");
        break;

    case Mission::GOTO_POSITION:
        LOG_MISSION(this->get_logger(), "[MISSION INFO] -> GOTO_POSITION");
        break;

    case Mission::DO_PROCESS:
        LOG_MISSION(this->get_logger(), "[MISSION INFO] -> DO_PROCESS ");
        break;

    default:
        LOG_ERROR(this->get_logger(), "[MISSION ERROR] Unknown mission state!");
        break;
    }

    // 2. Perform any initial calculations required for the NEW mission state
    initial_calculations_before_mission();
}
