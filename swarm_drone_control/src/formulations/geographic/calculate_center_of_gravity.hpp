#pragma once
#ifndef CALCULATE_CENTER_OF_GRAVITY_HPP
#define CALCULATE_CENTER_OF_GRAVITY_HPP

#include <vector>
#include <iostream>
#include <stdexcept>
#include "../../interfaces/vehicle_positions.hpp"

class CalculateCenterofGravity
{
public:
    CalculateCenterofGravity() = default;
    ~CalculateCenterofGravity() = default;

    template <typename T1, typename T2>
    static T1 calculate_cog(const std::vector<T2> &positions)
    {
        const size_t size_of_positions = positions.size();

        if (size_of_positions == 0)
        {
            throw std::invalid_argument("Position list is empty");
        }

        T1 cog{};
        for (const auto &pos : positions)
        {
            cog.lat += pos.lat;
            cog.lon += pos.lon;
        }

        const float size_float = static_cast<float>(size_of_positions);
        cog.lat /= size_float;
        cog.lon /= size_float;

        return cog;
    }
};

#endif // CALCULATE_CENTER_OF_GRAVITY_HPP