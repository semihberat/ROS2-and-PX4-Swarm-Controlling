#pragma once

#ifndef CALCULATE_OFFSET_FROM_CENTER_HPP
#define CALCULATE_OFFSET_FROM_CENTER_HPP

#include <vector>
#include <iostream>
#include <stdexcept>
#include <cmath>

#define PI 3.14159
#define EARTH_RADIUS 6371000 // in meters

#include "../../interfaces/vehicle_positions.hpp"

class CalculateOffsetsFromCenter{
    public:
        CalculateOffsetsFromCenter() = default;
        ~CalculateOffsetsFromCenter() = default;

        std::vector<VehicleVerticalPositions> calculate_offsets(const VehicleVerticalPositions& center, 
                                                                float offset_north,
                                                                float offset_east,
                                                                size_t num_positions){
            std::vector<VehicleVerticalPositions> offsets;
            offsets.reserve(num_positions);  // Performance optimization

            if(num_positions == 0){
                throw std::invalid_argument("Number of positions cannot be zero");
            }

            for(size_t i = 0; i < num_positions; i++){
                auto new_position = new_positions_after_offsets(center, static_cast<int16_t>(i), offset_north, offset_east);
                offsets.push_back(new_position);
            }
            
            return offsets;
        }

    private:
         VehicleVerticalPositions new_positions_after_offsets(const VehicleVerticalPositions& center, int16_t idx, float offset_north, float offset_east){
              VehicleVerticalPositions new_position{};
              new_position.lat = center.lat + (offset_north * idx / EARTH_RADIUS) * (180 / PI);
              new_position.lon = center.lon + (offset_east * idx / EARTH_RADIUS) * (180 / PI) / cos(center.lat * PI / 180);
              return new_position;
         }
};

#endif // CALCULATE_OFFSET_FROM_CENTER_HPP
//31 10 2025