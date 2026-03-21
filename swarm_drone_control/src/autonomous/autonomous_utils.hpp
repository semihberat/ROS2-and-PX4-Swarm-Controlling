#ifndef AUTONOMOUS_UTILS_HPP
#define AUTONOMOUS_UTILS_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include "calculations/geographic.hpp"

namespace autonomous_utils
{
    constexpr double STOP_THRESHOLD_01 = 0.25;
    constexpr double STOP_THRESHOLD_001 = 0.01;
    constexpr float P_GAIN = 0.9f;

    inline bool NeighborVerification(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &neighbors,
                                     std::function<double(const px4_msgs::msg::VehicleGlobalPosition &)> func)
    {
        return std::all_of(neighbors.begin(), neighbors.end(), [func](const px4_msgs::msg::VehicleGlobalPosition &pos)
                           { return std::abs(func(pos)) <= STOP_THRESHOLD_01; });
    }

    inline void calculate_all_distances(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &neighbors,
                                        const px4_msgs::msg::VehicleGlobalPosition &main_position,
                                        std::vector<DLatDLon> &out_distances)
    {
        out_distances.clear();
        for (const auto &neighbor_pos : neighbors)
        {
            out_distances.push_back(geo::calculate_distance<DLatDLon>(main_position.lat, main_position.lon,
                                                                      neighbor_pos.lat, neighbor_pos.lon));
        }
    }

    template <typename T>
    inline T calculate_velocity(T error, T max_vel, T gain = P_GAIN)
    {
        return std::clamp<T>(error * gain, -max_vel, max_vel);
    }

    // combine_positions removed to let callers use pre-allocated vectors

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
        Waypoints::SharedPtr wp_msg_;
        int current_index_{0};

    public:
        WaypointManager()
        {
            wp_msg_ = std::make_shared<Waypoints>();
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

        const px4_msgs::msg::VehicleGlobalPosition *jump_to(int index)
        {
            if (wp_msg_ && index >= 0 && index < static_cast<int>(wp_msg_->waypoints.size()))
            {
                current_index_ = index;
                return current();
            }
            return nullptr;
        }
    };
}

#endif // AUTONOMOUS_UTILS_HPP
