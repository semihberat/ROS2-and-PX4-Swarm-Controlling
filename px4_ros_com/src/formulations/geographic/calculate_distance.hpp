#pragma once

#ifndef CALCULATE_DISTANCE_HPP  
#define CALCULATE_DISTANCE_HPP

#include <vector>
#include <iostream>
#include <stdexcept>
#include <cmath>

# define PI 3.14159
# define EARTH_RADIUS 6371000 // in meters

class CalculateDistance{
    public:
        CalculateDistance() = default;
        ~CalculateDistance() = default;

        std::tuple<double, double, double> calculate_distance(double lat1, double lon1, double lat2, double lon2){
            double dlat = (lat2 - lat1) * (PI / 180.0);
            double dlon = (lon2 - lon1) * (PI / 180.0);
            double dlat_meter = dlat * EARTH_RADIUS;
            double dlon_meter = dlon * EARTH_RADIUS * cos(lat1 * (PI / 180.0));
            double distance = sqrt(dlat_meter * dlat_meter + dlon_meter * dlon_meter);

            return std::make_tuple(dlat_meter, dlon_meter, distance);
        }
};

#endif // CALCULATE_DISTANCE_HPP
//31 10 2025