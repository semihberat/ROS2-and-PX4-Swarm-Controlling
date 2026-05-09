
#include "drone_core/drone_core.hpp"

void DroneCore::callbackDroneCommands(const std::shared_ptr<custom_interfaces::srv::DroneCommands::Request> request,
                                      std::shared_ptr<custom_interfaces::srv::DroneCommands::Response> response)
{
    const bool targets_this_drone = std::any_of(request->ids.begin(), request->ids.end(), [this](uint8_t id)
                                                { return id == this->sys_id || id == 0; });
    if (!targets_this_drone)
        return;
    this->req_ = request;

    this->publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f); // stop the drone before executing the command
    switch (request->command)
    {
    case custom_interfaces::srv::DroneCommands::Request::HOLD:
        response->message = "[SUCCESS][HOLD] command received.";
        response->success = true;
        break;
    case custom_interfaces::srv::DroneCommands::Request::TAKEOFF:
        response->message = "[SUCCESS][TAKEOFF] command received.";
        response->success = true;
        break;
    case custom_interfaces::srv::DroneCommands::Request::LAND:
        // Implement LAND command logic here
        response->message = "[SUCCESS][LAND] command received.";
        response->success = true;
        break;
    case custom_interfaces::srv::DroneCommands::Request::WAYPOINT:

        if (!is_armed_)
        {
            RCLCPP_INFO(this->get_logger(), "Drone is not armed!");
            this->req_->command = custom_interfaces::srv::DroneCommands::Request::HOLD;
            response->message = "[ERROR][WAYPOINT] Drone is not armed.";
            response->success = false;
            return;
        };
        response->message = "[SUCCESS][WAYPOINT] command received.";
        response->success = true;
        break;

    default:
        response->message = "[ERROR] Unknown command received.";
        response->success = false;
        break;
    }
}
