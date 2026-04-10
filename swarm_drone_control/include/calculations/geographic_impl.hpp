#ifndef GEOGRAPHIC_IMPL_HPP
#define GEOGRAPHIC_IMPL_HPP

#include "geographic.hpp"

namespace geo
{

    // Sabit - Bumi'nin rata-rata yarıçapı (meter)
    constexpr double EARTH_RADIUS_M = 6371000.0;

    // Toleransi default (meter)
    constexpr double DEFAULT_TOLERANCE = 0.001;

    inline Distance diff_points(const custom_interfaces::msg::GeoPoint &point1,
                                const custom_interfaces::msg::GeoPoint &point2)
    {
        // Latitude ve Longitude farklarını derajacıkta hesapla
        double d_lat = point2.lat - point1.lat;
        double d_lon = point2.lon - point1.lon;

        // Derajaya dönüştür (Haversine için)
        double lat1_rad = point1.lat * M_PI / 180.0;
        double lon1_rad = point1.lon * M_PI / 180.0;
        double lat2_rad = point2.lat * M_PI / 180.0;
        double lon2_rad = point2.lon * M_PI / 180.0;

        double dlat_rad = lat2_rad - lat1_rad;
        double dlon_rad = lon2_rad - lon1_rad;

        // Haversine formülü ile yatay mesafeyi hesapla
        double a = std::sin(dlat_rad / 2.0) * std::sin(dlat_rad / 2.0) +
                   std::cos(lat1_rad) * std::cos(lat2_rad) *
                       std::sin(dlon_rad / 2.0) * std::sin(dlon_rad / 2.0);
        double c = 2.0 * std::asin(std::sqrt(a));
        double d_horizontal = EARTH_RADIUS_M * c;

        // Altitude farkı
        double d_alt = std::fabs(point2.alt - point1.alt);

        return Distance{
            d_lat + DEFAULT_TOLERANCE,
            d_lon + DEFAULT_TOLERANCE,
            d_horizontal + DEFAULT_TOLERANCE,
            d_alt + DEFAULT_TOLERANCE};
    }

    inline double calculate_bearing(const custom_interfaces::msg::GeoPoint &from,
                                    const custom_interfaces::msg::GeoPoint &to)
    {
        double lat1_rad = deg_to_rad(from.lat);
        double lon1_rad = deg_to_rad(from.lon);
        double lat2_rad = deg_to_rad(to.lat);
        double lon2_rad = deg_to_rad(to.lon);

        double dlon_rad = lon2_rad - lon1_rad;

        double y = std::sin(dlon_rad) * std::cos(lat2_rad);
        double x = std::cos(lat1_rad) * std::sin(lat2_rad) -
                   std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dlon_rad);

        double bearing_rad = std::atan2(y, x);
        return bearing_rad; // Radyan cinsinden döndür
    }

} // namespace geo

#endif // GEOGRAPHIC_IMPL_HPP
