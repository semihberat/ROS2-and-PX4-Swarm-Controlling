#ifndef AUTONOMUS_UTILS_HPP
#define AUTONOMUS_UTILS_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include "calculations/geographic.hpp"

namespace autonomus_utils
{
    constexpr double STOP_THRESHOLD_01 = 0.1;
    constexpr double STOP_THRESHOLD_001 = 0.01;
    constexpr float P_GAIN = 0.5f;

    inline bool NeighborVerification(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &neighbors,
                                     std::function<double(const px4_msgs::msg::VehicleGlobalPosition &)> func)
    {
        return std::all_of(neighbors.begin(), neighbors.end(), [func](const px4_msgs::msg::VehicleGlobalPosition &pos)
                           { return std::abs(func(pos)) <= STOP_THRESHOLD_001; });
    }

    inline std::vector<DLatDLon> all_distances(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &neighbors, const px4_msgs::msg::VehicleGlobalPosition &main_position)
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

    template <typename T>
    inline T calculate_velocity(T error, T max_vel, T gain = P_GAIN)
    {
        return std::clamp<T>(error * gain, -max_vel, max_vel);
    }

    inline std::vector<px4_msgs::msg::VehicleGlobalPosition> combine_positions(
        const std::vector<px4_msgs::msg::VehicleGlobalPosition> &neighbors,
        const px4_msgs::msg::VehicleGlobalPosition &main_position)
    {
        std::vector<px4_msgs::msg::VehicleGlobalPosition> positions = neighbors;
        positions.push_back(main_position);
        return positions;
    }

    inline px4_msgs::msg::VehicleGlobalPosition find_nearest_vehicle_to_target(
        const std::vector<px4_msgs::msg::VehicleGlobalPosition> &all_positions,
        const px4_msgs::msg::VehicleGlobalPosition &target_waypoint)
    {
        if (all_positions.empty())
            return target_waypoint; // fallback

        px4_msgs::msg::VehicleGlobalPosition nearest = all_positions.front();
        double min_dist = geo::calculate_distance<DLatDLon>(target_waypoint.lat, target_waypoint.lon, nearest.lat, nearest.lon).distance;

        for (const auto &pos : all_positions)
        {
            double dist = geo::calculate_distance<DLatDLon>(target_waypoint.lat, target_waypoint.lon, pos.lat, pos.lon).distance;
            if (dist < min_dist)
            {
                min_dist = dist;
                nearest = pos;
            }
        }
        return nearest;
    }

    inline double calculate_target_bearing_for_drone(
        const px4_msgs::msg::VehicleGlobalPosition &cog,
        const px4_msgs::msg::VehicleGlobalPosition &nearest_vehicle,
        const px4_msgs::msg::VehicleGlobalPosition &target_waypoint,
        const px4_msgs::msg::VehicleGlobalPosition &main_position)
    {
        double bearing_to_nearest = geo::calculate_bearing<px4_msgs::msg::VehicleGlobalPosition>(cog, nearest_vehicle);
        double bearing_to_target = geo::calculate_bearing<px4_msgs::msg::VehicleGlobalPosition>(cog, target_waypoint);
        double formation_rotation_angle = spatial::WrapAngleToPi(bearing_to_target - bearing_to_nearest);

        double current_drone_bearing_to_cog = geo::calculate_bearing<px4_msgs::msg::VehicleGlobalPosition>(cog, main_position);

        return spatial::WrapAngleToPi(current_drone_bearing_to_cog + formation_rotation_angle);
    }

    class WaypointManager
    {
    private:
        custom_interfaces::msg::Waypoints::SharedPtr wp_msg_;
        int current_index_{0};

    public:
        WaypointManager()
        {
            wp_msg_ = std::make_shared<custom_interfaces::msg::Waypoints>();
        }

        void set_waypoints(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &list)
        {
            if (wp_msg_)
            {
                wp_msg_->waypoints = list;
                current_index_ = 0;
            }
        }

        const px4_msgs::msg::VehicleGlobalPosition *current() const
        {
            if (wp_msg_ && current_index_ >= 0 && current_index_ < static_cast<int>(wp_msg_->waypoints.size()))
            {
                return &wp_msg_->waypoints[current_index_];
            }
            return nullptr;
        }

        const px4_msgs::msg::VehicleGlobalPosition *next()
        {
            if (wp_msg_ && current_index_ + 1 < static_cast<int>(wp_msg_->waypoints.size()))
            {
                current_index_++;
                return current();
            }
            if (wp_msg_)
                current_index_ = wp_msg_->waypoints.size();
            return nullptr;
        }

        bool is_finished() const
        {
            if (!wp_msg_)
                return true;
            return current_index_ >= static_cast<int>(wp_msg_->waypoints.size());
        }

        void reset()
        {
            current_index_ = 0;
        }
    };
}

#endif // AUTONOMUS_UTILS_HPP
