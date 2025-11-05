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

            double a = sin(dlat / 2) * sin(dlat / 2) +
                       cos(lat1 * (PI / 180.0)) * cos(lat2 * (PI / 180.0)) *
                       sin(dlon / 2) * sin(dlon / 2);
            double c = 2 * atan2(sqrt(a), sqrt(1 - a));
            double distance = EARTH_RADIUS * c;

            return std::make_tuple(dlat, dlon, distance);
        }
};

#endif // CALCULATE_DISTANCE_HPP
//31 10 2025