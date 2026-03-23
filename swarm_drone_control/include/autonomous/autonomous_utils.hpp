#ifndef AUTONOMOUS_UTILS_HPP
#define AUTONOMOUS_UTILS_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <memory>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <custom_interfaces/msg/waypoints.hpp>
#include "calculations/geographic.hpp"
#include "calculations/spatial.hpp"

namespace autonomous_utils
{
    constexpr double STOP_THRESHOLD_01 = 0.25;
    constexpr double STOP_THRESHOLD_001 = 0.01;
    constexpr float P_GAIN = 0.9f;

    inline bool NeighborVerification(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &neighbors,
                                     std::function<double(const px4_msgs::msg::VehicleGlobalPosition &)> func);

    inline void calculate_all_distances(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &neighbors,
                                        const px4_msgs::msg::VehicleGlobalPosition &main_position,
                                        std::vector<DLatDLon> &out_distances);

    template <typename T>
    inline T calculate_velocity(T error, T max_vel, T gain = P_GAIN);

    inline double calculate_target_bearing_for_drone(
        const px4_msgs::msg::VehicleGlobalPosition &cog,
        const px4_msgs::msg::VehicleGlobalPosition &nearest_vehicle,
        const px4_msgs::msg::VehicleGlobalPosition &target_waypoint,
        const px4_msgs::msg::VehicleGlobalPosition &main_position);

    class WaypointManager
    {
    private:
        custom_interfaces::msg::Waypoints::SharedPtr wp_msg_;
        int current_index_{0};

    public:
        WaypointManager();
        void set_waypoints(const std::vector<px4_msgs::msg::VehicleGlobalPosition> &list);
        const px4_msgs::msg::VehicleGlobalPosition *current() const;
        const px4_msgs::msg::VehicleGlobalPosition *next();
        bool is_finished() const;
        void reset();
        const px4_msgs::msg::VehicleGlobalPosition *jump_to(int index);
    };
}

#include "autonomous_utils_impl.hpp"

#endif // AUTONOMOUS_UTILS_HPP
