/**
 * @file geographic_impl.hpp
 * @brief Implementation of geographic calculation functions
 * @note This file is automatically included by geographic.hpp
 */

#ifndef GEOGRAPHIC_IMPL_HPP
#define GEOGRAPHIC_IMPL_HPP

namespace geo
{
    template <typename T1, typename T2>
    T1 calculate_cog(const std::vector<T2> &positions)
    {
        if (positions.empty())
        {
            throw std::invalid_argument("Position list is empty");
        }

        // Convert to Cartesian (ECEF) coordinates and average
        double x_sum = 0.0, y_sum = 0.0, z_sum = 0.0;

        for (const auto &pos : positions)
        {
            double lat_rad = deg_to_rad(pos.lat);
            double lon_rad = deg_to_rad(pos.lon);

            x_sum += std::cos(lat_rad) * std::cos(lon_rad);
            y_sum += std::cos(lat_rad) * std::sin(lon_rad);
            z_sum += std::sin(lat_rad);
        }

        double n = static_cast<double>(positions.size());
        double x_avg = x_sum / n;
        double y_avg = y_sum / n;
        double z_avg = z_sum / n;

        // Convert back to lat/lon
        T1 cog{};
        cog.lon = rad_to_deg(std::atan2(y_avg, x_avg));
        double hyp = std::sqrt(x_avg * x_avg + y_avg * y_avg);
        cog.lat = rad_to_deg(std::atan2(z_avg, hyp));

        return cog;
    }

    template <typename T>
    T calculate_distance(double lat1, double lon1, double lat2, double lon2)
    {
        T result;

        // Convert to radians
        double lat1_rad = deg_to_rad(lat1);
        double lat2_rad = deg_to_rad(lat2);
        double dlat_rad = deg_to_rad(lat2 - lat1);
        double dlon_rad = deg_to_rad(lon2 - lon1);

        // Haversine formula for great-circle distance
        double a = std::sin(dlat_rad / 2.0) * std::sin(dlat_rad / 2.0) +
                   std::cos(lat1_rad) * std::cos(lat2_rad) *
                       std::sin(dlon_rad / 2.0) * std::sin(dlon_rad / 2.0);
        double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

        result.distance = EARTH_RADIUS * c;

        // NED frame projection using average latitude
        double lat_avg = deg_to_rad((lat1 + lat2) / 2.0);
        result.dlat_meter = dlat_rad * EARTH_RADIUS;
        result.dlon_meter = dlon_rad * EARTH_RADIUS * std::cos(lat_avg);

        return result;
    }

    inline double normalize_angle(double angle)
    {
        // Normalize to [-π, π] range
        while (angle > PI_VAL)
            angle -= 2.0 * PI_VAL;
        while (angle < -PI_VAL)
            angle += 2.0 * PI_VAL;
        return angle;
    }

    inline double calculate_bearing(double lat1, double lon1, double lat2, double lon2)
    {
        // Convert to radians
        double lat1_rad = deg_to_rad(lat1);
        double lat2_rad = deg_to_rad(lat2);
        double dlon_rad = deg_to_rad(lon2 - lon1);

        // Forward azimuth formula
        double y = std::sin(dlon_rad) * std::cos(lat2_rad);
        double x = std::cos(lat1_rad) * std::sin(lat2_rad) -
                   std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dlon_rad);

        // atan2 already returns [-π, π]
        return std::atan2(y, x);
    }

    template <typename T>
    inline double calculate_bearing(const T &p_origin, const T &p_target)
    {
        return calculate_bearing(p_origin.lat, p_origin.lon, p_target.lat, p_target.lon);
    }
    /**
     * @brief Calculate a new position based on an origin and a target bearing
     *        using Spherical Earth calculations (Haversine/Great Circle).
     * @tparam T Position type with lat and lon (e.g., LatLon)
     * @param current The current position of the drone
     * @param reference The reference (center) point
     * @param target_bearing_rad The desired bearing from the reference to the final position
     * @return The calculated target position in geographic coordinates
     */
    template <typename T>
    T calculate_new_point(const T &current, const T &reference, double target_bearing_rad)
    {
        // 1. Calculate the distance (radius) from center to our current position in meters
        // This calculates the great-circle distance on the spherical earth
        double radius_meters = calculate_distance<DLatDLon>(
            reference.lat, reference.lon,
            current.lat, current.lon).distance;

        // 2. We want a new position that is at the exact same 'radius_meters' distance 
        // away from the 'reference' point, but heading in the 'target_bearing_rad' direction.
        // We use the exact destination_point formulation that properly wraps lat/lon 
        // across the globe using the Earth's radius.
        
        double lat1_rad = deg_to_rad(reference.lat);
        double lon1_rad = deg_to_rad(reference.lon);

        // angular distance in radians across the sphere
        double angular_dist = radius_meters / EARTH_RADIUS;

        // Spherical earth destination coordinate formulas
        double lat2_rad = std::asin(std::sin(lat1_rad) * std::cos(angular_dist) +
                                    std::cos(lat1_rad) * std::sin(angular_dist) * std::cos(target_bearing_rad));
        
        double lon2_rad = lon1_rad + std::atan2(std::sin(target_bearing_rad) * std::sin(angular_dist) * std::cos(lat1_rad),
                                                std::cos(angular_dist) - std::sin(lat1_rad) * std::sin(lat2_rad));

        T new_point{};
        new_point.lat = rad_to_deg(lat2_rad);
        new_point.lon = rad_to_deg(lon2_rad);
        return new_point;
    }

    template <typename T>
    std::vector<T> calculate_offsets(const T &center,
                                     double offset_north,
                                     double offset_east,
                                     size_t num_positions)
    {
        std::vector<T> offsets;
        offsets.reserve(num_positions);

        if (num_positions == 0)
        {
            throw std::invalid_argument("Number of positions cannot be zero");
        }

        for (size_t i = 0; i < num_positions; i++)
        {
            T new_position = center;
            
            // Apply North offset
            if (offset_north != 0.0) {
                double bearing_north = (offset_north > 0) ? 0.0 : PI_VAL;
                
                // We create a temporary point offset_north distance away purely in latitude for the new logic
                T north_pos = new_position;
                north_pos.lat += rad_to_deg(offset_north / EARTH_RADIUS); // Convert meters to lat degrees approx for the offset
                new_position = calculate_new_point(north_pos, new_position, bearing_north);
            }
            
            // Apply East offset
            if (offset_east != 0.0) {
                double bearing_east = (offset_east > 0) ? PI_VAL / 2.0 : -PI_VAL / 2.0;
                
                // We create a temporary point offset_east distance away purely in longitude
                T east_pos = new_position;
                east_pos.lon += rad_to_deg(offset_east / (EARTH_RADIUS * std::cos(deg_to_rad(new_position.lat))));
                new_position = calculate_new_point(east_pos, new_position, bearing_east);
            }
            
            offsets.push_back(new_position);
        }

        return offsets;
    }

} // namespace geo

#endif // GEOGRAPHIC_IMPL_HPP
