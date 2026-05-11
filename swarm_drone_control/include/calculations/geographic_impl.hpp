#ifndef GEOGRAPHIC_IMPL_HPP
#define GEOGRAPHIC_IMPL_HPP

#include "geographic.hpp"

namespace geo
{

    // Sabit - Bumi'nin rata-rata yarıçapı (meter)
    constexpr double EARTH_RADIUS_M = 6371000.0;

    // Toleransi default (meter)
    constexpr double DEFAULT_TOLERANCE = 0.0001;

    inline Distance diff_points(const custom_interfaces::msg::GeoPoint &point1,
                                const custom_interfaces::msg::GeoPoint &point2)
    {
        // Derece farklarını radyana çevir
        double lat1_rad = point1.lat * M_PI / 180.0;
        double lat2_rad = point2.lat * M_PI / 180.0;
        double dlat_rad = (point2.lat - point1.lat) * M_PI / 180.0;
        double dlon_rad = (point2.lon - point1.lon) * M_PI / 180.0;

        // --- Metre Cinsinden Farklar (Yerel Yaklaşım) ---
        // Enlem farkı metre: Her derece sabit yaklaşık 111,319 metredir.
        double d_lat_m = dlat_rad * EARTH_RADIUS_M;

        // Boylam farkı metre: Bulunulan enleme göre daralır (cos(lat) ile çarpılır).
        double d_lon_m = dlon_rad * EARTH_RADIUS_M * std::cos(lat1_rad);

        // --- Haversine ile Toplam Yatay Mesafe ---
        double a = std::sin(dlat_rad / 2.0) * std::sin(dlat_rad / 2.0) +
                   std::cos(lat1_rad) * std::cos(lat2_rad) *
                       std::sin(dlon_rad / 2.0) * std::sin(dlon_rad / 2.0);
        double c = 2.0 * std::asin(std::sqrt(a));
        double d_horizontal = EARTH_RADIUS_M * c;

        // Altitude farkı
        double d_alt = std::fabs(point2.alt - point1.alt);

        return Distance{
            d_lat_m + DEFAULT_TOLERANCE,
            d_lon_m + DEFAULT_TOLERANCE,
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

    inline custom_interfaces::msg::GeoPoint centroid(const std::vector<custom_interfaces::msg::DroneInfo::SharedPtr> &drones)
    {
        double x_sum = 0.0;
        double y_sum = 0.0;
        double z_sum = 0.0;
        size_t valid_count = 0;

        for (const auto &drone : drones)
        {
            if (drone == nullptr || (std::abs(drone->geo_point.lat) < 1e-5 && std::abs(drone->geo_point.lon) < 1e-5))
            {
                continue;
            }

            valid_count++;
            double lat_rad = drone->geo_point.lat * M_PI / 180.0;
            double lon_rad = drone->geo_point.lon * M_PI / 180.0;

            x_sum += std::cos(lat_rad) * std::cos(lon_rad);
            y_sum += std::cos(lat_rad) * std::sin(lon_rad);
            z_sum += std::sin(lat_rad);
        }

        if (valid_count == 0)
            return custom_interfaces::msg::GeoPoint();

        x_sum /= valid_count;
        y_sum /= valid_count;
        z_sum /= valid_count;

        double lon = std::atan2(y_sum, x_sum);
        double hyp = std::sqrt(x_sum * x_sum + y_sum * y_sum);
        double lat = std::atan2(z_sum, hyp);

        return custom_interfaces::msg::GeoPoint()
            .set__lat(lat * 180.0 / M_PI)
            .set__lon(lon * 180.0 / M_PI)
            .set__alt(0.0f);
    }

    inline custom_interfaces::msg::GeoPoint after_offset(const custom_interfaces::msg::GeoPoint &point, geo::Distance offset)
    {
        double lat_rad = point.lat * M_PI / 180.0;

        // Enlem için radyan cinsinden değişim
        double dlat_rad = offset.d_lat / EARTH_RADIUS_M;
        double new_lat_rad = lat_rad + dlat_rad;

        // Daha hassas boylam değişimi için iki enlemin ortalamasını kullanıyoruz (Meridian convergence)
        double mean_lat_rad = (lat_rad + new_lat_rad) / 2.0;
        double cos_mean_lat = std::cos(mean_lat_rad);
        if (std::abs(cos_mean_lat) < 1e-10)
            cos_mean_lat = 1e-10; // Kutuplara yaklaşma durumu için koruma

        double dlon_rad = offset.d_lon / (EARTH_RADIUS_M * cos_mean_lat);

        return custom_interfaces::msg::GeoPoint()
            .set__lat(point.lat + (dlat_rad * 180.0 / M_PI))
            .set__lon(point.lon + (dlon_rad * 180.0 / M_PI))
            .set__alt(point.alt + offset.d_alt);
    }

} // namespace geo

#endif // GEOGRAPHIC_IMPL_HPP
