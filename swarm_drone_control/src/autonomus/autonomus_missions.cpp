#include "autonomus.hpp"

// SWARM MEMBER PATH PLANNER IMPLEMENTATIONS

// Execute synchronized takeoff in formation
void SwarmMemberPathPlanner::formational_takeoff()
{
    const auto wp = waypoint_manager_.current();
    if (!wp)
        return;

    double z_error = current_altitude + wp->alt;
    current_commands.z_vel = autonomus_utils::calculate_velocity<double>(z_error, desired_velocities.z_vel);
    if (std::abs(z_error) <= autonomus_utils::STOP_THRESHOLD_01)
    {

        if (autonomus_utils::NeighborVerification(this->current_neighbors_info_->neighbor_positions,
                                                  [wp](const px4_msgs::msg::VehicleGlobalPosition &pos)
                                                  {
                                                      return -pos.alt - wp->alt;
                                                  }))
        {
            next_step();
            return;
        }
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
    }
    publish_trajectory_setpoint(0.0, 0.0, current_commands.z_vel, 0.0);
}

// Rotate formation while maintaining relative positions
void SwarmMemberPathPlanner::formational_rotation()
{
    auto d = geo::calculate_distance<DLatDLon>(this->current_neighbors_info_->main_position.lat,
                                               this->current_neighbors_info_->main_position.lon,
                                               swarm_positions.circular_position.lat, swarm_positions.circular_position.lon);
    current_commands.v_lat = autonomus_utils::calculate_velocity<double>(d.dlat_meter, desired_velocities.v_lat);
    current_commands.v_lon = autonomus_utils::calculate_velocity<double>(d.dlon_meter, desired_velocities.v_lon);
    current_commands.yaw_vel = autonomus_utils::calculate_velocity<double>(d.dlat_meter, desired_velocities.yaw_vel);

    // Check Neighbors
    if (std::abs(d.distance) < autonomus_utils::STOP_THRESHOLD_01)
    {
        next_step();
        return;
    }
    publish_trajectory_setpoint(current_commands.v_lat, current_commands.v_lon, 0.0, current_commands.yaw_vel);
}

// Navigate to target position
void SwarmMemberPathPlanner::goto_position()
{
    auto d = geo::calculate_distance<DLatDLon>(this->current_neighbors_info_->main_position.lat,
                                               this->current_neighbors_info_->main_position.lon,
                                               swarm_positions.target_after_offset.lat, swarm_positions.target_after_offset.lon);
    current_commands.v_lat = autonomus_utils::calculate_velocity<double>(d.dlat_meter, desired_velocities.v_lat);
    current_commands.v_lon = autonomus_utils::calculate_velocity<double>(d.dlon_meter, desired_velocities.v_lon);
    current_commands.bearing = geo::calculate_bearing<VehicleGlobalPosition>(this->current_neighbors_info_->main_position, swarm_positions.target_after_offset);
    current_commands.yaw_vel = autonomus_utils::calculate_velocity<double>(current_commands.bearing, desired_velocities.yaw_vel);

    current_commands.v_lat += collision_bias.vlat;
    current_commands.v_lon += collision_bias.vlon;

    if (std::abs(d.distance) <= autonomus_utils::STOP_THRESHOLD_01)
    {

       
        next_step();
        return;
    }

    publish_trajectory_setpoint(current_commands.v_lat, current_commands.v_lon, 0.0, current_commands.yaw_vel);
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
        LOG_WARN(this->get_logger(), "[MISSION WARN] __No neighbor information available. Cannot proceed to next mission step.");
        return;
    }

    initialize_positioned_drones();

    switch (this->current_mission)
    {
    case Mission::FORMATIONAL_TAKEOFF:
    {

        this->current_mission = Mission::FORMATIONAL_ROTATION;
        LOG_MISSION(this->get_logger(), "[MISSION INFO] __FORMATIONAL_ROTATION");
        break;
    }

    case Mission::FORMATIONAL_ROTATION:
    {
        const auto wp = waypoint_manager_.current();
        if (!wp)
            return;

        auto radius_from_cog = geo::calculate_distance<DLatDLon>(swarm_positions.cog.lat, swarm_positions.cog.lon,
                                                                 this->current_neighbors_info_->main_position.lat,
                                                                 this->current_neighbors_info_->main_position.lon);

        swarm_positions.target_after_offset = geo::calculate_offsets<VehicleGlobalPosition>(*wp,
                                                                                            radius_from_cog.dlat_meter, radius_from_cog.dlon_meter, 1)[0];

        initial_n_distances = autonomus_utils::all_distances(this->current_neighbors_info_->neighbor_positions, this->current_neighbors_info_->main_position);

        this->current_mission = Mission::GOTO_POSITION;
        LOG_MISSION(this->get_logger(), "[MISSION INFO] __GOTO_POSITION");
        break;
    }

    case Mission::GOTO_POSITION:
    {
        // Hedefe ulaşıldı, bir sonraki waypoint'e geç
        const auto *next_wp = waypoint_manager_.next();
        
        if (next_wp != nullptr)
        {
            // Yeni waypoint varsa tekrar rotasyon ve gidiş adımlarına başla
            this->current_mission = Mission::FORMATIONAL_ROTATION;
            LOG_MISSION(this->get_logger(), "[MISSION INFO] __FORMATIONAL_ROTATION (Next Waypoint)");
            
            // Yeni waypoint için hedefleri tekrar hesapla
            initial_calculations_before_mission();
        }
        else
        {
            // Tüm waypointler bittiyse görev sonu (veya işlem) durumuna geç
            this->current_mission = Mission::DO_PROCESS;
            LOG_MISSION(this->get_logger(), "[MISSION INFO] __DO_PROCESS");
        }
        break;
    }
    case Mission::DO_PROCESS:
    case Mission::END_TASK:
        break;

    default:
        LOG_ERROR(this->get_logger(), "[MISSION ERROR] Unknown mission state! Resetting to initial mission.");
        this->current_mission = Mission::FORMATIONAL_TAKEOFF;
        break;
    }
}

void SwarmMemberPathPlanner::initial_calculations_before_mission()
{
    const auto wp = waypoint_manager_.current();
    if (!wp)
        return;

    // 1. Center of Gravities
    std::vector<VehicleGlobalPosition> all_positions = autonomus_utils::combine_positions(
        this->current_neighbors_info_->neighbor_positions,
        this->current_neighbors_info_->main_position);

    swarm_positions.cog = geo::calculate_cog<VehicleGlobalPosition>(all_positions);

    // 2. Nearest vehicle to the target position
    swarm_positions.nearest_vehicle = autonomus_utils::find_nearest_vehicle_to_target(
        all_positions,
        *wp);

    // 3. Bearing control for THIS DRONE based on entire formation
    double my_new_target_bearing = autonomus_utils::calculate_target_bearing_for_drone(
        swarm_positions.cog,
        swarm_positions.nearest_vehicle,
        *wp,
        this->current_neighbors_info_->main_position);

    // 4. Calculate this drone's new spot on the rotated formation circle
    swarm_positions.circular_position = geo::calculate_new_point<VehicleGlobalPosition>(
        this->current_neighbors_info_->main_position,
        swarm_positions.cog,
        my_new_target_bearing);
}

void SwarmMemberPathPlanner::initialize_positioned_drones()
{
    // Reset in_target requests for all drones
    positioned_drones_.clear();
    positioned_drones_.reserve(this->current_neighbors_info_->neighbor_positions.size() + 1);

    auto request = std::make_shared<InTarget::Request>();
    request->drone_id = 0;
    request->is_in_position = false;

    // Add neighbors
    for (size_t i = 0; i < this->current_neighbors_info_->neighbor_positions.size(); ++i)
    {
        positioned_drones_.push_back(request);
    }

    // Add self
    positioned_drones_.push_back(request);
}