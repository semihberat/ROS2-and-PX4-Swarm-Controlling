#include <drone_core/drone_core.hpp>

void DroneCore::state_cycle()
{
    if (!this->req_)
    {
        return;
    }

    switch (this->req_->command)
    {
    case custom_interfaces::srv::DroneCommands::Request::TAKEOFF:
        this->takeoff(this->service_parameters_.takeoff_alt);
        break;
    case custom_interfaces::srv::DroneCommands::Request::LAND:
        this->land();
        break;
    case custom_interfaces::srv::DroneCommands::Request::HOLD:
        this->hold();
        break;
    case custom_interfaces::srv::DroneCommands::Request::WAYPOINT:
        this->goto_waypoints(this->req_->geo_points);
        break;

    case custom_interfaces::srv::DroneCommands::Request::GRID:
        break;
    default:
        this->publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
        break;
    }
}