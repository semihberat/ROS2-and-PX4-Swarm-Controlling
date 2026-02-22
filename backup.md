```cpp
#include "swarm_drone_control/swarm_member_path_planner.hpp"

// === CALLBACK FUNCTIONS ===

void SwarmMemberPathPlanner::timer_callback()
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        publish_trajectory_setpoint(target_vlat, target_vlon, 0.0, 0.0);
    }
}

void SwarmMemberPathPlanner::path_planner_callback(const NeighborsInfo::SharedPtr msg)
{
    if (verification_count < verification_count_max)
    {
        verification_count++;
        all_positions = msg->neighbor_positions;
        all_positions.push_back(msg->main_position);

        auto center_of_gravity = geo::calculate_cog<VehicleVerticalPositions, VehicleGlobalPosition>(all_positions);
        auto offsets = geo::calculate_offsets(center_of_gravity, offset_lat, offset_lon, all_positions.size());
        auto matched_position = offsets[msg->main_id - 1];

        target_position_ = matched_position;
    }

    VectoralDistance distance_left = geo::calculate_distance<VectoralDistance>(
        msg->main_position.lat, msg->main_position.lon,
        target_position_.lat, target_position_.lon);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_vlat = distance_left.dlat_meter;
        target_vlon = distance_left.dlon_meter;
    }
    for (const auto &neighbor : msg->neighbor_positions)
    {
        auto uav_distance = geo::calculate_distance<VectoralDistance>(
            msg->main_position.lat, msg->main_position.lon,
            neighbor.lat, neighbor.lon);

        if (uav_distance.distance <= collision_tolerance_m)
        {
            target_vlon = -uav_distance.dlon_meter + target_vlon;
            target_vlat = -uav_distance.dlat_meter + target_vlat;
        }
    }
}
```