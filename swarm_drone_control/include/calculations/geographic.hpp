#ifndef GEOGRAPHIC_HPP
#define GEOGRAPHIC_HPP

#include "custom_interfaces/msg/geo_point.hpp"
#include <cmath>
#include <tuple>

namespace geo
{

    /**
     * @struct Distance
     * @brief Struktura untuk menyimpan perbedaan antara dua titik geografis
     */
    struct Distance
    {
        double d_lat; // Latitude farkı (derajacıkta)
        double d_lon; // Longitude farkı (derajacıkta)
        double d;     // 3B bileşke mesafe (meter)
        double d_alt; // Altitude farkı (meter)
    };

    /**
     * @brief Menghitung perbedaan antara dua titik geografis dalam meter
     *
     * @param point1 Titik pertama (lat, lon, alt)
     * @param point2 Titik kedua (lat, lon, alt)
     * @return Distance Struktur berisi d_lat, d_lon, d (3B bileşke), dan d_alt
     */
    Distance diff_points(const custom_interfaces::msg::GeoPoint &point1,
                         const custom_interfaces::msg::GeoPoint &point2);

    double calculate_bearing(const custom_interfaces::msg::GeoPoint &from,
                             const custom_interfaces::msg::GeoPoint &to);

    inline double deg_to_rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    inline double rad_to_deg(double rad)
    {
        return rad * 180.0 / M_PI;
    }

} // namespace geo

#include "geographic_impl.hpp"

#endif // GEOGRAPHIC_HPP
