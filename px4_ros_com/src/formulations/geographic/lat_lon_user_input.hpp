#pragma once

#ifndef LAT_LON_USER_INPUT_HPP
#define LAT_LON_USER_INPUT_HPP

#include "../../interfaces/vehicle_positions.hpp"

class LatLonUserInput{

    private:
        VehicleVerticalPositions user_position{};

    public:
        LatLonUserInput() = default;
        ~LatLonUserInput() = default;
        
        VehicleVerticalPositions set_user_input(double lat, double lon){
            user_position.lat = lat;
            user_position.lon = lon;
            return user_position;
        }

        VehicleVerticalPositions get_user_input(){
            return user_position;
        }
};

#endif // LAT_LON_USER_INPUT_HPP
