/**
 * @file geographic.hpp
 * @brief Geographic calculations and formulations for drone navigation
 *
 * This header consolidates all geographic calculation functions under the 'geo' namespace.
 * Usage: geo::calculate_cog(), geo::calculate_distance(), geo::calculate_offsets(), geo::calculate_bearing()
 */

#ifndef GEOGRAPHIC_HPP
#define GEOGRAPHIC_HPP

#include <vector>
#include <iostream>
#include <stdexcept>
#include <cmath>

// === DATA STRUCTURES ===

/**
 * @brief Vehicle vertical position (latitude/longitude)
 */
struct LatLon
{
    double lat = 0.0;
    double lon = 0.0;
};

/**
 * @brief Vectoral distance structure (meters)
 */
struct DLatDLon
{
    double dlat_meter = 0.0;
    double dlon_meter = 0.0;
    double distance = 0.0;
};

// === GEOGRAPHIC NAMESPACE ===

namespace geo
{
    // Constants
    constexpr double PI_VAL = 3.14159265358979323846;
    constexpr double EARTH_RADIUS = 6371000.0; // meters

    /**
     * @brief Convert degrees to radians
     * @param deg Angle in degrees
     * @return Angle in radians
     */
    inline constexpr double deg_to_rad(double deg) { return deg * (PI_VAL / 180.0); }

    /**
     * @brief Convert radians to degrees
     * @param rad Angle in radians
     * @return Angle in degrees
     */
    inline constexpr double rad_to_deg(double rad) { return rad * (180.0 / PI_VAL); }

    /**
     * @brief Calculate center of gravity from a list of positions
     * @tparam T1 Return type (e.g., VehicleVerticalPositions)
     * @tparam T2 Input position type
     * @param positions Vector of positions
     * @return Center of gravity position
     * @throws std::invalid_argument if position list is empty
     */
    template <typename T1, typename T2>
    T1 calculate_cog(const std::vector<T2> &positions);

    /**
     * @brief Calculate distance between two geographic coordinates
     * @tparam T Return type (e.g., VectoralDistance)
     * @param lat1 Latitude of first point (degrees)
     * @param lon1 Longitude of first point (degrees)
     * @param lat2 Latitude of second point (degrees)
     * @param lon2 Longitude of second point (degrees)
     * @return Vectoral distance with dlat_meter, dlon_meter, and total distance
     */
    template <typename T>
    T calculate_distance(double lat1, double lon1, double lat2, double lon2);

    /**
     * @brief Normalize angle to [-π, π] range
     * @param angle Angle in radians
     * @return Normalized angle in [-π, π]
     */
    double normalize_angle(double angle);

    /**
     * @brief Calculate bearing angle from point1 to point2
     * @param lat1 Starting latitude (degrees)
     * @param lon1 Starting longitude (degrees)
     * @param lat2 Target latitude (degrees)
     * @param lon2 Target longitude (degrees)
     * @return Bearing angle in radians [-π, π] (0=North, π/2=East, -π/2=West)
     */
    double calculate_bearing(double lat1, double lon1, double lat2, double lon2);

    /**
     * @brief Calculate bearing angle from point1 to point2
     * @tparam T Position type with lat and lon (e.g., LatLon)
     * @param p_origin Starting position
     * @param p_target Target position
     * @return Bearing angle in radians [-π, π] (0=North, π/2=East, -π/2=West)
     */
    template <typename T>
    double calculate_bearing(const T &p_origin, const T &p_target);

    template <typename T>
    T calculate_new_point(const T &current, const T &reference, double target_bearing_rad);

    /**
     * @brief Calculate offset positions from a center point
     * @tparam T Position type (e.g., VehicleVerticalPositions)
     * @param center Center position
     * @param offset_north North offset in meters
     * @param offset_east East offset in meters
     * @param num_positions Number of positions to calculate
     * @return Vector of offset positions
     * @throws std::invalid_argument if num_positions is zero
     */
    template <typename T>
    std::vector<T> calculate_offsets(const T &center,
                                     double offset_north,
                                     double offset_east,
                                     size_t num_positions);

} // namespace geo

// Include implementation
#include "geographic_impl.hpp"

#endif // GEOGRAPHIC_HPP
