#include "autonomous.hpp"
#include <map>
#include <limits>

// TARGET SYNCRONIZATION IMPLEMENTATIONS

void SwarmMemberPathPlanner::in_target_callback(const InTarget::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    drones_in_target_[msg->drone_id] = msg->in_target;
}

bool SwarmMemberPathPlanner::check_all_drones_in_target()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (int i = 1; i <= total_drones_; ++i)
    {
        if (!drones_in_target_[i])
            return false;
    }
    return true;
}

bool SwarmMemberPathPlanner::verify_in_target_state(double current_error, double target_threshold)
{
    double hysteresis_threshold = target_threshold * 3.0; // 0.25 -> 0.75m distance tolerance

    bool currently_in;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        currently_in = drones_in_target_[sys_id_];
    }

    bool is_inside = currently_in;

    if (!currently_in && current_error <= target_threshold)
    {
        is_inside = true;
    }
    else if (currently_in && current_error > hysteresis_threshold)
    {
        is_inside = false;
    }

    bool state_changed = (is_inside != currently_in);

    // Keep publishing if we are inside to ensure all drones know, or at least publish the change if exiting
    if (state_changed || is_inside)
    {
        InTarget msg;
        msg.drone_id = sys_id_;
        msg.in_target = is_inside;
        in_target_publisher_->publish(msg);

        std::lock_guard<std::mutex> lock(data_mutex_);
        drones_in_target_[sys_id_] = is_inside;
    }

    if (is_inside)
    {
        return check_all_drones_in_target();
    }
    return false;
}

void SwarmMemberPathPlanner::reset_in_target_status()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (int i = 1; i <= total_drones_; ++i)
    {
        drones_in_target_[i] = false;
    }
}

// Update basic horizontal velocities based on target distance
void SwarmMemberPathPlanner::update_velocity()
{
    current_commands.v_lat = autonomous_utils::calculate_velocity<double>(this->target_distance_.dlat_meter, desired_velocities.v_lat);
    current_commands.v_lon = autonomous_utils::calculate_velocity<double>(this->target_distance_.dlon_meter, desired_velocities.v_lon);
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
        this->current_neighbors_info_->neighbor_positions.end());
    this->all_positions.push_back(this->current_neighbors_info_->main_position);

    swarm_positions.cog = geo::calculate_cog<VehicleGlobalPosition>(this->all_positions);
}

void SwarmMemberPathPlanner::elect_leader()
{
    // Harita oluştur: Drone ID -> Güncel Küresel Pozisyon
    std::map<uint8_t, VehicleGlobalPosition> id_to_pos_map;
    id_to_pos_map[this->current_neighbors_info_->main_id] = this->current_neighbors_info_->main_position;

    for (size_t i = 0; i < this->current_neighbors_info_->neighbor_ids.size(); ++i)
    {
        id_to_pos_map[this->current_neighbors_info_->neighbor_ids[i]] = this->current_neighbors_info_->neighbor_positions[i];
    }

    // Eğer lider daha önce seçilmemişse veya kopmuşsa (artık ağda yoksa), YENİ LİDER SEÇ!
    if (swarm_positions.leader_id == 0 || id_to_pos_map.find(swarm_positions.leader_id) == id_to_pos_map.end())
    {
        double min_dist_to_target = std::numeric_limits<double>::max();
        uint8_t selected_leader_id = this->current_neighbors_info_->main_id;

        for (const auto &pair : id_to_pos_map)
        {
            uint8_t drone_id = pair.first;
            const auto &position = pair.second;

            double dist = geo::calculate_distance<DLatDLon>(position.lat, position.lon,
                                                            current_wp_->lat, current_wp_->lon)
                              .distance;

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

    swarm_positions.leader_vehicle = id_to_pos_map[swarm_positions.leader_id];
}

void SwarmMemberPathPlanner::calculate_mission_specific_targets()
{
    switch (this->current_mission)
    {
    case Mission::FORMATIONAL_ROTATION:
    {
        swarm_positions.target_bearing_from_cog = autonomous_utils::calculate_target_bearing_for_drone(
            swarm_positions.cog,
            swarm_positions.leader_vehicle,
            *current_wp_,
            this->current_neighbors_info_->main_position);

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
