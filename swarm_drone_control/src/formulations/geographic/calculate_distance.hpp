#pragma once

#ifndef CALCULATE_DISTANCE_HPP
#define CALCULATE_DISTANCE_HPP

#include <vector>
#include <iostream>
#include <stdexcept>
#include <cmath>

class CalculateDistance
{
public:
    CalculateDistance() = default;
    ~CalculateDistance() = default;

    static constexpr float PI_VAL = 3.14159f;
    static constexpr float EARTH_RADIUS = 6371000.0f; // in meters

    template <typename T>
    static T calculate_distance(float lat1, float lon1, float lat2, float lon2)
    {
        T result;
        float dlat = (lat2 - lat1) * (PI_VAL / 180.0f);
        float dlon = (lon2 - lon1) * (PI_VAL / 180.0f);
        result.dlat_meter = dlat * EARTH_RADIUS;
        result.dlon_meter = dlon * EARTH_RADIUS * cos(lat1 * (PI_VAL / 180.0f));
        result.distance = sqrt(result.dlat_meter * result.dlat_meter + result.dlon_meter * result.dlon_meter);

        return result;
    }
};

#endif // CALCULATE_DISTANCE_HPP
       // 31 10 2025