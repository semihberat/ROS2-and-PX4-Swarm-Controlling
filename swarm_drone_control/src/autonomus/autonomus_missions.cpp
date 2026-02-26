#include "autonomus.hpp"
#define STOP_THRESHOLD 0.01

// Utilities
bool NeighborVerification(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &neighbors,
                          std::function<double(const px4_msgs::msg::VehicleGlobalPosition &)> func)
{
    return std::all_of(neighbors.begin(), neighbors.end(), [func](const px4_msgs::msg::VehicleGlobalPosition &pos)
                       { return std::abs(func(pos)) <= STOP_THRESHOLD; });
}

std::vector<DLatDLon> all_distances(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &neighbors, const VehicleGlobalPosition &main_position)
{
    std::vector<DLatDLon> distances;
    distances.reserve(neighbors.size());
    for (const auto &neighbor_pos : neighbors)
    {
        distances.push_back(geo::calculate_distance<DLatDLon>(main_position.lat, main_position.lon,
                                                              neighbor_pos.lat, neighbor_pos.lon));
    }
    return distances;
}

// SWARM MEMBER PATH PLANNER IMPLEMENTATIONS

// Execute synchronized takeoff in formation
void SwarmMemberPathPlanner::formational_takeoff()
{
    double z_error = current_altitude + current_waypoint_.alt;
    double z_vel = std::clamp<float>(z_error * 0.5, -desired_z_vel, desired_z_vel);
    if (std::abs(z_error) <= STOP_THRESHOLD)
    {

        if (NeighborVerification(this->current_neighbors_info_->neighbor_positions,
                                 [this](const px4_msgs::msg::VehicleGlobalPosition &pos)
                                 {
                                     return -pos.alt - current_waypoint_.alt;
                                 }))
        {
            next_step();
            return;
        }
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
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
    double yaw_vel = std::clamp<float>(d.dlat_meter * 0.5, -desired_yaw_vel, desired_yaw_vel);

    // Check Neighbors
    if (std::abs(d.distance) <= STOP_THRESHOLD)
    {
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
        next_step();
        return;
    }
    publish_trajectory_setpoint(v_lat, v_lon, 0.0, yaw_vel);
}

// Navigate to target position
void SwarmMemberPathPlanner::goto_position()
{
    auto d = geo::calculate_distance<DLatDLon>(this->current_neighbors_info_->main_position.lat,
                                               this->current_neighbors_info_->main_position.lon,
                                               target_after_offset.lat, target_after_offset.lon);
    double v_lat = std::clamp<float>(d.dlat_meter * 0.5, -desired_v_lat, desired_v_lat);
    double v_lon = std::clamp<float>(d.dlon_meter * 0.5, -desired_v_lon, desired_v_lon);

    if (std::abs(d.distance) <= STOP_THRESHOLD)
    {
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
        next_step();
        return;
    }

    publish_trajectory_setpoint(v_lat, v_lon, 0.0, 0.0);
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
    initial_calculations_before_mission();

    if (current_neighbors_info_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(), "[MISSION WARN] __No neighbor information available. Cannot proceed to next mission step.");
        return;
    }

    switch (this->current_mission)
    {
    case Mission::FORMATIONAL_TAKEOFF:
    {
        RCLCPP_INFO(this->get_logger(), "[MISSION INFO] __TAKEOFF");
        this->current_mission = Mission::FORMATIONAL_ROTATION;
        break;
    }

    case Mission::FORMATIONAL_ROTATION:
    {
        RCLCPP_INFO(this->get_logger(), "[MISSION INFO] __FORMATIONAL_ROTATION");
        auto radius_from_cog = geo::calculate_distance<DLatDLon>(cog.lat, cog.lon,
                                                                 this->current_neighbors_info_->main_position.lat,
                                                                 this->current_neighbors_info_->main_position.lon);

        target_after_offset = geo::calculate_offsets<VehicleGlobalPosition>(current_waypoint_,
                                                                            radius_from_cog.dlat_meter, radius_from_cog.dlon_meter, 1)[0];

        this->current_mission = Mission::GOTO_POSITION;
        break;
    }

    case Mission::GOTO_POSITION:
    {
        RCLCPP_INFO(this->get_logger(), "[MISSION INFO] __GOTO_POSITION");
        this->current_mission = Mission::DO_PROCESS;
        break;
    }
    case Mission::DO_PROCESS:
    case Mission::END_TASK:
        break;

    default:
        RCLCPP_ERROR(this->get_logger(), "[MISSION ERROR] Unknown mission state! Resetting to initial mission.");
        this->current_mission = Mission::FORMATIONAL_TAKEOFF;
        break;
    }
}

void SwarmMemberPathPlanner::initial_calculations_before_mission()
{
    // Center of Gravities
    std::vector<VehicleGlobalPosition> positions = this->current_neighbors_info_->neighbor_positions;
    positions.push_back(this->current_neighbors_info_->main_position);
    cog = geo::calculate_cog<VehicleGlobalPosition>(positions);

    // Nearest vehicle to the target position
    nearest_vehicle = this->current_neighbors_info_->main_position;
    double min_dist = geo::calculate_distance<DLatDLon>(current_waypoint_.lat, current_waypoint_.lon,
                                                        this->current_neighbors_info_->main_position.lat,
                                                        this->current_neighbors_info_->main_position.lon)
                          .distance;
    for (const auto &neighbor_pos : current_neighbors_info_->neighbor_positions)
    {
        double dist = geo::calculate_distance<DLatDLon>(current_waypoint_.lat, current_waypoint_.lon,
                                                        neighbor_pos.lat, neighbor_pos.lon)
                          .distance;
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
}