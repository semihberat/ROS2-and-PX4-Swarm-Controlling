#include <drone_core/drone_core.hpp>

void DroneCore::state_cycle()
{
    using namespace custom_interfaces::srv;
    if (!this->req_)
    {
        return;
    }

    switch (this->req_->command)
    {
    case DroneCommands::Request::TAKEOFF:
        this->takeoff(this->service_parameters_.takeoff_alt);
        break;
    case DroneCommands::Request::LAND:
        this->land();
        break;
    case DroneCommands::Request::HOLD:
        this->hold();
        break;
    case DroneCommands::Request::WAYPOINT:
        this->goto_waypoints(this->req_->geo_points);

        break;

    default:
        this->publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
        break;
    }
}