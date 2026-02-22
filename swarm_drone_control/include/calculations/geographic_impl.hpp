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
            double lat_rad = pos.lat * (PI_VAL / 180.0);
            double lon_rad = pos.lon * (PI_VAL / 180.0);

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
        cog.lon = std::atan2(y_avg, x_avg) * (180.0 / PI_VAL);
        double hyp = std::sqrt(x_avg * x_avg + y_avg * y_avg);
        cog.lat = std::atan2(z_avg, hyp) * (180.0 / PI_VAL);

        return cog;
    }

    template <typename T>
    T calculate_distance(double lat1, double lon1, double lat2, double lon2)
    {
        T result;

        // Convert to radians
        double lat1_rad = lat1 * (PI_VAL / 180.0);
        double lat2_rad = lat2 * (PI_VAL / 180.0);
        double dlat_rad = (lat2 - lat1) * (PI_VAL / 180.0);
        double dlon_rad = (lon2 - lon1) * (PI_VAL / 180.0);

        // Haversine formula for great-circle distance
        double a = std::sin(dlat_rad / 2.0) * std::sin(dlat_rad / 2.0) +
                   std::cos(lat1_rad) * std::cos(lat2_rad) *
                       std::sin(dlon_rad / 2.0) * std::sin(dlon_rad / 2.0);
        double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

        result.distance = EARTH_RADIUS * c;

        // NED frame projection using average latitude
        double lat_avg = (lat1 + lat2) / 2.0 * (PI_VAL / 180.0);
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
        double lat1_rad = lat1 * (PI_VAL / 180.0);
        double lat2_rad = lat2 * (PI_VAL / 180.0);
        double dlon_rad = (lon2 - lon1) * (PI_VAL / 180.0);

        // Forward azimuth formula
        double y = std::sin(dlon_rad) * std::cos(lat2_rad);
        double x = std::cos(lat1_rad) * std::sin(lat2_rad) -
                   std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dlon_rad);

        // atan2 already returns [-π, π]
        return std::atan2(y, x);
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
            T new_position{};
            new_position.lat = center.lat + (offset_north * i / EARTH_RADIUS) * (180.0 / PI_VAL);
            new_position.lon = center.lon + (offset_east * i / EARTH_RADIUS) * (180.0 / PI_VAL) / std::cos(center.lat * PI_VAL / 180.0);
            offsets.push_back(new_position);
        }

        return offsets;
    }

} // namespace geo

#endif // GEOGRAPHIC_IMPL_HPP
