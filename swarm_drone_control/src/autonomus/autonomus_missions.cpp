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
    auto d = geo::calculate_distance<DLatDLon>(this->current_neighbors_info_->main_position.lat, 
        this->current_neighbors_info_->main_position.lon,
        circular_position.lat, circular_position.lon);
    double v_lat = std::clamp<float>(d.dlat_meter * 0.5, -desired_v_lat, desired_v_lat);
    double v_lon = std::clamp<float>(d.dlon_meter * 0.5, -desired_v_lon, desired_v_lon);

    if(std::abs(v_lat) < 0.01 && std::abs(v_lon) < 0.01)
    {
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
        next_step();
        return;
    }

    publish_trajectory_setpoint(v_lat, v_lon, 0.0, 0.0);    
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
    // Center of Gravities
    std::vector<VehicleGlobalPosition> positions = this->current_neighbors_info_->neighbor_positions;
    positions.push_back(this->current_neighbors_info_->main_position);
    cog = geo::calculate_cog<VehicleGlobalPosition>(positions);

    // Nearest vehicle to the target position
    nearest_vehicle = this->current_neighbors_info_->main_position;
    double min_dist = geo::calculate_distance<DLatDLon>(current_waypoint_.lat, current_waypoint_.lon,
                                                               this->current_neighbors_info_->main_position.lat,
                                                               this->current_neighbors_info_->main_position.lon).distance;  
    for (const auto &neighbor_pos : current_neighbors_info_->neighbor_positions)
    {
        double dist = geo::calculate_distance<DLatDLon>(current_waypoint_.lat, current_waypoint_.lon,
                                                                neighbor_pos.lat, neighbor_pos.lon).distance;  
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_vehicle = neighbor_pos;
        }
    }
    // Bearing control for the entire formation
    double bearing_to_nearest = geo::calculate_bearing<VehicleGlobalPosition>(cog, nearest_vehicle);
    double bearing_to_target = geo::calculate_bearing<VehicleGlobalPosition>(cog, current_waypoint_);
    double formation_rotation_angle = spatial::WrapAngleToPi(bearing_to_target - bearing_to_nearest);

    // Specifically for THIS DRONE, calculate its current bearing from the COG
    double current_drone_bearing_to_cog = geo::calculate_bearing<VehicleGlobalPosition>(cog, this->current_neighbors_info_->main_position);
    
    // The target angle for THIS DRONE on the circle is its current bearing + the total formation rotation needed
    double my_new_target_bearing = spatial::WrapAngleToPi(current_drone_bearing_to_cog + formation_rotation_angle);

    // Calculate this drone's new spot on the rotated formation circle
    circular_position = geo::calculate_new_point<VehicleGlobalPosition>(
        this->current_neighbors_info_->main_position, 
        cog, 
        my_new_target_bearing);

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