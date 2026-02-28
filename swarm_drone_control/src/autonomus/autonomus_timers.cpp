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

    current_n_distances = autonomus_utils::all_distances(this->current_neighbors_info_->neighbor_positions, this->current_neighbors_info_->main_position);
    
    collision_bias.vlat = 0.0;
    collision_bias.vlon = 0.0;

    for (size_t i = 0; i < this->current_n_distances.size(); ++i)
    {
        double dlat_diff = this->current_n_distances[i].dlat_meter - this->initial_n_distances[i].dlat_meter;
        double dlon_diff = this->current_n_distances[i].dlon_meter - this->initial_n_distances[i].dlon_meter;
        if(dlat_diff > 1.0){
            collision_bias.vlat += dlat_diff;
        }
        if(dlon_diff > 1.0){
            collision_bias.vlon += dlon_diff;
        }
    }

    collision_bias.vlat *= 0.5;
    collision_bias.vlon *= 0.5;
    collision_bias.vlat = std::clamp<double>(collision_bias.vlat, -1.5, 1.5);
    collision_bias.vlon = std::clamp<double>(collision_bias.vlon, -1.5, 1.5);
}
