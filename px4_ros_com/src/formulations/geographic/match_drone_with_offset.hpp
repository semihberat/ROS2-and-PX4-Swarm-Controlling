#pragma once

#ifndef MATCH_DRONE_WITH_OFFSET_HPP
#define MATCH_DRONE_WITH_OFFSET_HPP

#include <vector>
#include <iostream>
#include <stdexcept>
#include <cmath>

#include <unordered_map>

#include "calculate_distance.hpp"
#include "../../interfaces/vehicle_positions.hpp"

class MatchDroneWithOffset{
    public:
        MatchDroneWithOffset() = default;
        ~MatchDroneWithOffset() = default;
        //C++ dilinde map ve unordered map kullanımı konusuna bakılacak
        //Sonrasında geliştirmeye müsait bir hale gelecektir
        
        //I will use std::map type function 
        template<typename T>
        VehicleVerticalPositions match(const T& vehicle_position, const std::vector<VehicleVerticalPositions>& offset_positions){
                double max_distance = 0.0;
                VehicleVerticalPositions matched_position;
                for (const auto& offset_position: offset_positions){
                    double dlat, dlon, distance;
                    std::tie(dlat, dlon, distance) = CalculateDistance().calculate_distance(
                        vehicle_position.lat, vehicle_position.lon,
                        offset_position.lat, offset_position.lon
                    );
                    if (distance > max_distance){
                        max_distance = distance;
                        matched_position = offset_position;
                    }
                    
                }
                return matched_position;
        }
};

#endif // MATCH_DRONES_WITH_OFFSETS_HPP
//31 10 2025