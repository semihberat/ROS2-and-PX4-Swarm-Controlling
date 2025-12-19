#pragma once

#ifndef CALCULATE_DISTANCE_HPP  
#define CALCULATE_DISTANCE_HPP

#include <vector>
#include <iostream>
#include <stdexcept>
#include <cmath>

# define PI 3.14159
# define EARTH_RADIUS 6371000 // in meters

#include "../../interfaces/vectoral_distance.hpp"

class CalculateDistance{
    public:
        CalculateDistance() = default;
        ~CalculateDistance() = default;

        template<typename T>
        T calculate_distance(double lat1, double lon1, double lat2, double lon2){
            T result;
            double dlat = (lat2 - lat1) * (PI / 180.0);
            double dlon = (lon2 - lon1) * (PI / 180.0);
            result.dlat_meter = dlat * EARTH_RADIUS;
            result.dlon_meter = dlon * EARTH_RADIUS * cos(lat1 * (PI / 180.0));
            result.distance = sqrt(result.dlat_meter * result.dlat_meter + result.dlon_meter * result.dlon_meter);

            return result;
        }
};

#endif // CALCULATE_DISTANCE_HPP
//31 10 2025