#include "autonomus.hpp"

// Mission state machine - called every 100ms
void SwarmMemberPathPlanner::state_cycle_callback()
{
    if (this->current_neighbors_info_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for neighbors info...");
        return;
    }

    switch (this->current_mission)
    {
    case Mission::FORMATIONAL_TAKEOFF:

        formational_takeoff();
        break;
    case Mission::FORMATIONAL_ROTATION:
        formational_rotation();
        break;
    case Mission::GOTO_POSITION:
        goto_position();
        break;
    case Mission::DO_PROCESS:
        do_process();
        break;
    case Mission::END_TASK:
        end_task();
        break;
    }
}

void SwarmMemberPathPlanner::collision_avoidance()
{
    if (this->current_neighbors_info_ == nullptr || this->initial_n_distances.empty())
        return;

    if (ignore_collision_counter > 0)
    {
        ignore_collision_counter--;
        collision_bias.vlat *= collision_cfg_.ignored_bias_decay;
        collision_bias.vlon *= collision_cfg_.ignored_bias_decay;
        previous_collision_bias_ = collision_bias;
        return;
    }

    current_n_distances = autonomus_utils::all_distances(this->current_neighbors_info_->neighbor_positions, this->current_neighbors_info_->main_position);

    if (current_n_distances.empty())
    {
        return;
    }

    const size_t pair_count = std::min(current_n_distances.size(), initial_n_distances.size());
    if (pair_count == 0)
    {
        return;
    }

    double raw_bias_vlat = 0.0;
    double raw_bias_vlon = 0.0;
    double min_neighbor_distance = std::numeric_limits<double>::max();

    // Dinamik çarpışma eşiği (hıza bağlı)
    auto v = std::sqrt(current_commands.v_lat * current_commands.v_lat + current_commands.v_lon * current_commands.v_lon);
    double d_col_threshold = std::max(collision_cfg_.min_collision_distance, v * collision_cfg_.collision_speed_factor);

    for (size_t i = 0; i < pair_count; ++i)
    {
        double current_dist = this->current_n_distances[i].distance;
        min_neighbor_distance = std::min(min_neighbor_distance, current_dist);

        // 1. İTİCİ GÜÇ (Repulsive - Sadece çok yaklaşıldığında ve gerçekten tehlike varsa sertleşecek şekilde)
        if (current_dist < d_col_threshold)
        {
            double safe_dist = std::max(collision_cfg_.min_safe_distance, current_dist);
            double force_magnitude = calculate_repulsive_force(safe_dist, d_col_threshold);

            // Yönü komşunun tam zıttına (tersine) çeviriyoruz
            double dir_lat = -this->current_n_distances[i].dlat_meter / safe_dist;
            double dir_lon = -this->current_n_distances[i].dlon_meter / safe_dist;

            raw_bias_vlat += dir_lat * force_magnitude;
            raw_bias_vlon += dir_lon * force_magnitude;
        }
        else
        {
            // 2. ÇEKİCİ / HİZALAYICI GÜÇ (Formaliteden - Yavaş Formasyon Düzeltmesi)
            double dlat_diff = this->current_n_distances[i].dlat_meter - this->initial_n_distances[i].dlat_meter;
            double dlon_diff = this->current_n_distances[i].dlon_meter - this->initial_n_distances[i].dlon_meter;

            // Uzak komşulardan gelen formasyon düzeltmesini yumuşat.
            const double formation_weight = std::clamp(current_dist / (d_col_threshold + 1.0), 0.25, 1.0);

            raw_bias_vlat += apply_deadband(dlat_diff, collision_cfg_.deadband_dlat) * collision_cfg_.formation_gain * formation_weight;
            raw_bias_vlon += apply_deadband(dlon_diff, collision_cfg_.deadband_dlon) * collision_cfg_.formation_gain * formation_weight;
        }
    }

    // Ani bias sıçramalarını azaltmak için düşük geçiren filtre.
    constexpr double bias_alpha = 0.35;
    collision_bias.vlat = static_cast<float>(bias_alpha * raw_bias_vlat + (1.0 - bias_alpha) * previous_collision_bias_.vlat);
    collision_bias.vlon = static_cast<float>(bias_alpha * raw_bias_vlon + (1.0 - bias_alpha) * previous_collision_bias_.vlon);

    // Stabilite için sınırları tatlı bir aralıkta tutuyoruz
    collision_bias.vlat = std::clamp<double>(collision_bias.vlat, -collision_cfg_.bias_limit, collision_cfg_.bias_limit);
    collision_bias.vlon = std::clamp<double>(collision_bias.vlon, -collision_cfg_.bias_limit, collision_cfg_.bias_limit);

    previous_collision_bias_ = collision_bias;

    // Local minimum tespiti: komut var ama ilerleme yoksa, güvenli mesafede çarpışma önlemeyi kısa süre kapat.
    if (stuck_check_counter == 0)
    {
        last_checked_stuck_pos = this->current_neighbors_info_->main_position;
    }

    stuck_check_counter++;
    if (stuck_check_counter < collision_cfg_.stuck_check_cycles)
    {
        return;
    }

    const auto progress = geo::calculate_distance<DLatDLon>(
        last_checked_stuck_pos.lat,
        last_checked_stuck_pos.lon,
        this->current_neighbors_info_->main_position.lat,
        this->current_neighbors_info_->main_position.lon);

    last_checked_stuck_pos = this->current_neighbors_info_->main_position;
    stuck_check_counter = 0;

    if (v < collision_cfg_.min_command_speed_for_stuck)
    {
        return;
    }

    if (progress.distance > collision_cfg_.stuck_movement_threshold)
    {
        return;
    }

    if (min_neighbor_distance < collision_cfg_.safe_disable_distance)
    {
        return;
    }

    ignore_collision_counter = collision_cfg_.disable_collision_cycles;
    collision_bias.vlat = 0.0f;
    collision_bias.vlon = 0.0f;
    previous_collision_bias_ = collision_bias;

    RCLCPP_WARN(this->get_logger(),
                COLOR_YELLOW "[COLLISION WARN] Local minimum detected. Disabling collision avoidance for %d cycles (min neighbor distance: %.2f m)." COLOR_RESET,
                ignore_collision_counter,
                min_neighbor_distance);
}

double SwarmMemberPathPlanner::apply_deadband(double value, double deadband)
{
    if (std::abs(value) <= deadband)
    {
        return 0.0;
    }
    return (value > 0.0) ? (value - deadband) : (value + deadband);
}

double SwarmMemberPathPlanner::calculate_repulsive_force(double current_dist, double threshold) const
{
    const double dist_diff = threshold - current_dist;
    return collision_cfg_.repulsion_gain * (dist_diff * dist_diff);
}
