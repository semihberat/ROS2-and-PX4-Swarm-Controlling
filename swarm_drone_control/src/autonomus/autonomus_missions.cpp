#include "autonomus.hpp"

// TARGET SYNCRONIZATION IMPLEMENTATIONS

void SwarmMemberPathPlanner::in_target_callback(const custom_interfaces::msg::InTarget::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->in_target) {
        drones_in_target_[msg->drone_id] = true;
    }
}

bool SwarmMemberPathPlanner::check_all_drones_in_target()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (int i = 1; i <= total_drones_; ++i)
    {
        if (!drones_in_target_[i]) return false;
    }
    return true;
}

void SwarmMemberPathPlanner::reset_in_target_status()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (int i = 1; i <= total_drones_; ++i)
    {
        drones_in_target_[i] = false;
    }
}

// Execute synchronized takeoff in formation
void SwarmMemberPathPlanner::formational_takeoff()
{
    double z_error = current_altitude + current_wp_->alt;
    current_commands.z_vel = autonomus_utils::calculate_velocity<double>(z_error, desired_velocities.z_vel);
    
    if (std::abs(z_error) <= autonomus_utils::STOP_THRESHOLD_01)
    {
        custom_interfaces::msg::InTarget msg;
        msg.drone_id = sys_id_;
        msg.in_target = true;
        in_target_publisher_->publish(msg);
        
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            drones_in_target_[sys_id_] = true;
        }

        if (check_all_drones_in_target())
        {
            next_step();
            return;
        }
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
    }
    else
    {
        publish_trajectory_setpoint(0.0, 0.0, current_commands.z_vel, 0.0);
    }
}

// Update basic horizontal velocities based on target distance
void SwarmMemberPathPlanner::update_velocity()
{
    current_commands.v_lat = autonomus_utils::calculate_velocity<double>(this->target_distance_.dlat_meter, desired_velocities.v_lat);
    current_commands.v_lon = autonomus_utils::calculate_velocity<double>(this->target_distance_.dlon_meter, desired_velocities.v_lon);
}

// Rotate formation while maintaining relative positions
void SwarmMemberPathPlanner::formational_rotation()
{
    this->target_distance_ = geo::calculate_distance<DLatDLon>(this->current_neighbors_info_->main_position.lat,
                                               this->current_neighbors_info_->main_position.lon,
                                               swarm_positions.circular_position.lat, swarm_positions.circular_position.lon);
    
    update_velocity();
    current_commands.yaw_vel = autonomus_utils::calculate_velocity<double>(this->target_distance_.dlat_meter, desired_velocities.yaw_vel);

    // Check Neighbors
    if (std::abs(this->target_distance_.distance) < autonomus_utils::STOP_THRESHOLD_01)
    {
        custom_interfaces::msg::InTarget msg;
        msg.drone_id = sys_id_;
        msg.in_target = true;
        in_target_publisher_->publish(msg);
        
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            drones_in_target_[sys_id_] = true;
        }

        if (check_all_drones_in_target())
        {
            next_step();
            return;
        }
        
        // Wait gracefully
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
        return;
    }
    publish_trajectory_setpoint(current_commands.v_lat, current_commands.v_lon, 0.0, current_commands.yaw_vel);
}

// Apply computed collision avoidance bias to final target velocities
void SwarmMemberPathPlanner::apply_collision_bias()
{
    current_commands.v_lat += collision_bias.vlat;
    current_commands.v_lon += collision_bias.vlon;
}

// Navigate to target position
void SwarmMemberPathPlanner::goto_position()
{
    this->target_distance_ = geo::calculate_distance<DLatDLon>(this->current_neighbors_info_->main_position.lat,
                                               this->current_neighbors_info_->main_position.lon,
                                               swarm_positions.target_after_offset.lat, swarm_positions.target_after_offset.lon);
    
    update_velocity();
    current_commands.bearing = geo::calculate_bearing<VehicleGlobalPosition>(this->current_neighbors_info_->main_position, swarm_positions.target_after_offset);
    current_commands.yaw_vel = autonomus_utils::calculate_velocity<double>(current_commands.bearing, desired_velocities.yaw_vel);

    // Collision avoidance modifying speeds
    apply_collision_bias();

    if (std::abs(this->target_distance_.distance) <= autonomus_utils::STOP_THRESHOLD_01)
    {
        custom_interfaces::msg::InTarget msg;
        msg.drone_id = sys_id_;
        msg.in_target = true;
        in_target_publisher_->publish(msg);
        
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            drones_in_target_[sys_id_] = true;
        }

        if (check_all_drones_in_target())
        {
            next_step();
            return;
        }
        
        // Wait gracefully
        publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
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
    // 1. Advance mission state
    switch (this->current_mission)
    {
    case Mission::FORMATIONAL_TAKEOFF:
        this->current_mission = Mission::FORMATIONAL_ROTATION;
        LOG_MISSION(this->get_logger(), "[MISSION INFO] -> FORMATIONAL_ROTATION");
        break;

    case Mission::FORMATIONAL_ROTATION:
        this->current_mission = Mission::GOTO_POSITION;
        LOG_MISSION(this->get_logger(), "[MISSION INFO] -> GOTO_POSITION");
        break;

    case Mission::GOTO_POSITION:
        // Hedefe ulaşıldı, bir sonraki waypoint'e geç
        current_wp_ = waypoint_manager_.next();
        if (current_wp_)
        {
            this->current_mission = Mission::FORMATIONAL_ROTATION;
            LOG_MISSION(this->get_logger(), "[MISSION INFO] -> FORMATIONAL_ROTATION (Next Waypoint)");
        }
        else
        {
            this->current_mission = Mission::DO_PROCESS;
            LOG_MISSION(this->get_logger(), "[MISSION INFO] -> DO_PROCESS");
        }
        break;

    case Mission::DO_PROCESS:
    case Mission::END_TASK:
        break;

    default:
        LOG_ERROR(this->get_logger(), "[MISSION ERROR] Unknown mission state! Resetting...");
        this->current_mission = Mission::FORMATIONAL_TAKEOFF;
        break;
    }

    // 2. Perform any initial calculations required for the NEW mission state
    initial_calculations_before_mission();
}

void SwarmMemberPathPlanner::initial_calculations_before_mission()
{
    if (!current_wp_)
        return;

    reset_in_target_status();

    // --- ORTAK HESAPLAMALAR (Her Mission geçişinde güncellenmesi gereken taze veriler) ---
    
    // 1. O anki tüm swarm pozisyonları ve güncel ağırlık merkezi (CoG)
    this->all_positions.clear();
    this->all_positions.insert(
        this->all_positions.end(),
        this->current_neighbors_info_->neighbor_positions.begin(),
        this->current_neighbors_info_->neighbor_positions.end()
    );
    this->all_positions.push_back(this->current_neighbors_info_->main_position);

    swarm_positions.cog = geo::calculate_cog<VehicleGlobalPosition>(this->all_positions);

    // 2. Hedefe en yakın drone (Lider/Referans tespiti)
    swarm_positions.nearest_vehicle = autonomus_utils::find_nearest_vehicle_to_target(
        this->all_positions,
        *current_wp_);
        
    // 3. Çarpışma önleme veya formasyon koruma için o anki güncel mesafelerin (Snapshot) alınması
    autonomus_utils::calculate_all_distances(
        this->current_neighbors_info_->neighbor_positions, 
        this->current_neighbors_info_->main_position,
        this->initial_n_distances);

    // --- MISSION (GÖREV) ÖZELİNDEKİ HESAPLAMALAR ---
    switch (this->current_mission)
    {
    case Mission::FORMATIONAL_ROTATION:
    {
        // Bearing control for THIS DRONE based on entire formation
        swarm_positions.target_bearing_from_cog = autonomus_utils::calculate_target_bearing_for_drone(
            swarm_positions.cog,
            swarm_positions.nearest_vehicle,
            *current_wp_,
            this->current_neighbors_info_->main_position);

        // Calculate this drone's new spot on the rotated formation circle
        swarm_positions.circular_position = geo::calculate_new_point<VehicleGlobalPosition>(
            this->current_neighbors_info_->main_position,
            swarm_positions.cog,
            swarm_positions.target_bearing_from_cog);
        break;
    }

    case Mission::GOTO_POSITION:
    {
        swarm_positions.offset_from_cog = geo::calculate_distance<DLatDLon>(swarm_positions.cog.lat, swarm_positions.cog.lon,
                                                                 this->current_neighbors_info_->main_position.lat,
                                                                 this->current_neighbors_info_->main_position.lon);

        auto offset_result = geo::calculate_offsets<VehicleGlobalPosition>(*current_wp_,
                                                                                            swarm_positions.offset_from_cog.dlat_meter, swarm_positions.offset_from_cog.dlon_meter, 1)[0];
        swarm_positions.target_after_offset = offset_result;
        break;
    }
    
    default:
        break;
    }
}