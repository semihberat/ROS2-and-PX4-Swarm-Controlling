#ifndef SURVEY_PLAN_HPP
#define SURVEY_PLAN_HPP

#include <vector>
#include "custom_interfaces/msg/geo_point.hpp"

using custom_interfaces::msg::GeoPoint;

namespace geo
{
    struct Point
    {
        double x, y;
    };

    class GeoSurveyPlanner
    {
    public:
        // Lat/Lon'u yerel metre (X, Y) koordinatına çevirir
        Point project(GeoPoint target, GeoPoint origin);

        // Metre (X, Y) koordinatını tekrar Lat/Lon'a çevirir
        GeoPoint unproject(Point p, GeoPoint origin);

        // GPS poligon ve metre cinsinden spacing ile survey havada uçus yolu oluşturur
        std::vector<GeoPoint> generateGeoPath(const std::vector<GeoPoint> &geoPolygon, double spacingInMeters);

    private:
        // Kartezyen düzlemde havada uçus yolu hesaplaması
        std::vector<Point> calculateLocalPath(const std::vector<Point> &polygon, double spacing);
    };
}

#include "survey_plan_impl.hpp"

#endif // SURVEY_PLAN_HPP

/*
    Örnek Kullanım:

    // Survey alanı tanımla (GPS koordinatları)
    std::vector<GeoPoint> surveyPolygon;
    surveyPolygon.push_back({41.0082, 28.9784});  // Noktası 1
    surveyPolygon.push_back({41.0095, 28.9800});  // Noktası 2
    surveyPolygon.push_back({41.0100, 28.9770});  // Noktası 3
    surveyPolygon.push_back({41.0090, 28.9760});  // Noktası 4

    // Planner'ı oluştur
    geo::GeoSurveyPlanner planner;

    // 20 metrelik aralıklarla uçus yolunu hesapla
    std::vector<GeoPoint> flightPath = planner.generateGeoPath(surveyPolygon, 20.0);

    // Waypoint'leri iterate et
    for (const auto& wp : flightPath) {
        // Her waypoint'i kullan (lat, lon)
        // Örn: drone'a gönder, log'a yazıl, vs.
    }
*/