#pragma once

#ifndef VEHICLE_POSITIONS_HPP
#define VEHICLE_POSITIONS_HPP

class VehicleVerticalPositions {
public:
    double lat = 0.0;
    double lon = 0.0;
    
    // Default constructor
    VehicleVerticalPositions() = default;
    
    // Constructor with values
    VehicleVerticalPositions(double latitude, double longitude)
        : lat(latitude), lon(longitude) {}
};

#endif // VEHICLE_POSITIONS_HPP