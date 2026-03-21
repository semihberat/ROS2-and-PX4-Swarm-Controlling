#include "autonomous.hpp"
#include <map>
#include <limits>

// TARGET SYNCRONIZATION IMPLEMENTATIONS

void SwarmMemberPathPlanner::in_target_callback(const InTarget::SharedPtr msg)
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
    current_commands.z_vel = autonomous_utils::calculate_velocity<double>(z_error, desired_velocities.z_vel);
    
    if (std::abs(z_error) <= autonomous_utils::STOP_THRESHOLD_01)
    {
        InTarget msg;
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
    current_commands.v_lat = autonomous_utils::calculate_velocity<double>(this->target_distance_.dlat_meter, desired_velocities.v_lat);
    current_commands.v_lon = autonomous_utils::calculate_velocity<double>(this->target_distance_.dlon_meter, desired_velocities.v_lon);
}

// Rotate formation while maintaining relative positions
void SwarmMemberPathPlanner::formational_rotation()
{
    this->target_distance_ = geo::calculate_distance<DLatDLon>(this->current_neighbors_info_->main_position.lat,
                                               this->current_neighbors_info_->main_position.lon,
                                               swarm_positions.circular_position.lat, swarm_positions.circular_position.lon);
    
    update_velocity();
    current_commands.yaw_vel = autonomous_utils::calculate_velocity<double>(this->target_distance_.dlat_meter, desired_velocities.yaw_vel);

    // Check Neighbors
    if (std::abs(this->target_distance_.distance) < autonomous_utils::STOP_THRESHOLD_01)
    {
        InTarget msg;
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
    current_commands.yaw_vel = autonomous_utils::calculate_velocity<double>(current_commands.bearing, desired_velocities.yaw_vel);

    // Collision avoidance modifying speeds
    apply_collision_bias();

    if (std::abs(this->target_distance_.distance) <= autonomous_utils::STOP_THRESHOLD_01)
    {
        InTarget msg;
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

    // 1. Ortak Pozisyonları / Ağırlık Merkezini Hesapla
    calculate_swarm_positions();

    // 2. Dinamik Sürü Liderini Belirle veya Koru
    elect_leader();

    // 3. Anlık Mesafe Snapshot'ını Al (Çarpışma önleme referansı)
    autonomous_utils::calculate_all_distances(
        this->current_neighbors_info_->neighbor_positions, 
        this->current_neighbors_info_->main_position,
        this->initial_n_distances);

    // 4. Spesifik Göreve Yönelik Hesaplamaları Yap
    calculate_mission_specific_targets();
}

void SwarmMemberPathPlanner::calculate_swarm_positions()
{
    // 1. O anki tüm swarm pozisyonları ve güncel ağırlık merkezi (CoG)
    this->all_positions.clear();
    this->all_positions.insert(
        this->all_positions.end(),
        this->current_neighbors_info_->neighbor_positions.begin(),
        this->current_neighbors_info_->neighbor_positions.end()
    );
    this->all_positions.push_back(this->current_neighbors_info_->main_position);

    swarm_positions.cog = geo::calculate_cog<VehicleGlobalPosition>(this->all_positions);
}

void SwarmMemberPathPlanner::elect_leader()
{
    // =========================================================================
    // 2. LEADER SELECTION (Sürü Lideri Seçimi)
    //
    // Lider, hedefe en yakın olan drone olarak seçilir ve hedefe ulaşıncaya kadar
    // (veya sistemden kopana/düşene kadar) liderliğini korur. Tüm rotasyon ve
    // formasyon işlemleri bu lider referans alınarak gerçekleştirilir.
    // =========================================================================

    // Harita oluştur: Drone ID -> Güncel Küresel Pozisyon
    std::map<uint8_t, VehicleGlobalPosition> id_to_pos_map;
    id_to_pos_map[this->current_neighbors_info_->main_id] = this->current_neighbors_info_->main_position;
    
    for (size_t i = 0; i < this->current_neighbors_info_->neighbor_ids.size(); ++i) {
        id_to_pos_map[this->current_neighbors_info_->neighbor_ids[i]] = this->current_neighbors_info_->neighbor_positions[i];
    }

    // Eğer lider daha önce seçilmemişse veya kopmuşsa (artık ağda yoksa), YENİ LİDER SEÇ!
    if (swarm_positions.leader_id == 0 || id_to_pos_map.find(swarm_positions.leader_id) == id_to_pos_map.end())
    {
        double min_dist_to_target = std::numeric_limits<double>::max();
        uint8_t selected_leader_id = this->current_neighbors_info_->main_id;
        
        for (const auto& pair : id_to_pos_map)
        {
            uint8_t drone_id = pair.first;
            const auto& position = pair.second;
            
            double dist = geo::calculate_distance<DLatDLon>(position.lat, position.lon, 
                                                            current_wp_->lat, current_wp_->lon).distance;
                                                            
            // En yakın aracı önceliklendir. Eşitlik durumunda daha küçük ID'li drone kazanır (Çelişkiyi önler).
            if (dist < min_dist_to_target)
            {
                min_dist_to_target = dist;
                selected_leader_id = drone_id;
            }
            else if (dist == min_dist_to_target && drone_id < selected_leader_id)
            {
                selected_leader_id = drone_id;
            }
        }
        
        swarm_positions.leader_id = selected_leader_id;
        RCLCPP_INFO(this->get_logger(), COLOR_GREEN "[LEADER ELECTION] Drone %d is the new SWARM LEADER!" COLOR_RESET, swarm_positions.leader_id);
    }

    // Dinamik rotasyon ve formasyon hesabında kullanılması için liderin anlık konumunu kaydet
    swarm_positions.leader_vehicle = id_to_pos_map[swarm_positions.leader_id];
}



void SwarmMemberPathPlanner::calculate_mission_specific_targets()
{
    // --- MISSION (GÖREV) ÖZELİNDEKİ HESAPLAMALAR ---
    switch (this->current_mission)
    {
    case Mission::FORMATIONAL_ROTATION:
    {
        // Bearing control for THIS DRONE based on the Leader's perspective
        swarm_positions.target_bearing_from_cog = autonomous_utils::calculate_target_bearing_for_drone(
            swarm_positions.cog,
            swarm_positions.leader_vehicle,
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