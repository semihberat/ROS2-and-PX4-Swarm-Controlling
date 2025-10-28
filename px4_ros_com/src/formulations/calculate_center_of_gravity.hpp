#pragma once

#include <vector>
#include <iostream>
#include <stdexcept>
#include "../interfaces/vehicle_positions.hpp"
 
class CalculateCenterofGravity {
public:
    CalculateCenterofGravity() = default;
    ~CalculateCenterofGravity() = default;

    template <typename T> 
    VehicleVerticalPositions calculate_cog(const std::vector<T>& positions) {
        const size_t size_of_positions = positions.size();
        
        if (size_of_positions == 0) {
            throw std::invalid_argument("Position list is empty");
        }

        VehicleVerticalPositions cog{};
        for (const auto& pos : positions) {
            cog.lat += pos.lat;
            cog.lon += pos.lon;
        }
        
        const double size_double = static_cast<double>(size_of_positions);
        cog.lat /= size_double;
        cog.lon /= size_double;
        
        return cog;
    }
};