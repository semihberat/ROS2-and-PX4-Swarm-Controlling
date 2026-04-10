#ifndef SURVEY_PLAN_IMPL_HPP
#define SURVEY_PLAN_IMPL_HPP

#include <algorithm>
#include <cmath>
#include "survey_plan.hpp"

// Dünya yarıçapı (metre)
const double EARTH_RADIUS = 6378137.0;

namespace geo
{
    inline Point GeoSurveyPlanner::project(GeoPoint target, GeoPoint origin)
    {
        double x = (target.lon - origin.lon) * (EARTH_RADIUS * M_PI / 180.0) * cos(origin.lat * M_PI / 180.0);
        double y = (target.lat - origin.lat) * (EARTH_RADIUS * M_PI / 180.0);
        return {x, y};
    }

    inline GeoPoint GeoSurveyPlanner::unproject(Point p, GeoPoint origin)
    {
        double lat = origin.lat + (p.y / EARTH_RADIUS) * (180.0 / M_PI);
        double lon = origin.lon + (p.x / (EARTH_RADIUS * cos(origin.lat * M_PI / 180.0))) * (180.0 / M_PI);
        GeoPoint gp;
        gp.lat = lat;
        gp.lon = lon;
        return gp;
    }

    inline std::vector<GeoPoint> GeoSurveyPlanner::generateGeoPath(const std::vector<GeoPoint> &geoPolygon, double spacingInMeters)
    {
        if (geoPolygon.empty())
            return {};

        // İlk noktayı (0,0) orijini kabul edelim
        GeoPoint origin = geoPolygon[0];
        std::vector<Point> localPolygon;
        for (const auto &gp : geoPolygon)
        {
            localPolygon.push_back(project(gp, origin));
        }

        // --- Kartezyen düzlemde hesaplama yap ---
        std::vector<Point> localPath = calculateLocalPath(localPolygon, spacingInMeters);

        // --- Sonuçları tekrar Lat/Lon'a çevir ---
        std::vector<GeoPoint> geoPath;
        for (const auto &lp : localPath)
        {
            geoPath.push_back(unproject(lp, origin));
        }
        return geoPath;
    }

    inline std::vector<Point> GeoSurveyPlanner::calculateLocalPath(const std::vector<Point> &polygon, double spacing)
    {
        std::vector<Point> path;
        double minY = polygon[0].y, maxY = polygon[0].y;
        for (const auto &p : polygon)
        {
            minY = std::min(minY, p.y);
            maxY = std::max(maxY, p.y);
        }

        bool directionRight = true;
        for (double y = minY + (spacing / 2.0); y <= maxY; y += spacing)
        {
            std::vector<double> xInts;
            for (size_t i = 0; i < polygon.size(); ++i)
            {
                Point p1 = polygon[i], p2 = polygon[(i + 1) % polygon.size()];
                if ((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y))
                {
                    xInts.push_back(p1.x + (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y));
                }
            }
            std::sort(xInts.begin(), xInts.end());

            if (directionRight)
            {
                for (size_t i = 0; i < xInts.size(); i += 2)
                {
                    if (i + 1 < xInts.size())
                    {
                        path.push_back({xInts[i], y});
                        path.push_back({xInts[i + 1], y});
                    }
                }
            }
            else
            {
                for (int i = (int)xInts.size() - 1; i >= 0; i -= 2)
                {
                    if (i - 1 >= 0)
                    {
                        path.push_back({xInts[i], y});
                        path.push_back({xInts[i - 1], y});
                    }
                }
            }
            directionRight = !directionRight;
        }
        return path;
    }
}

#endif // SURVEY_PLAN_IMPL_HPP
