#include "uav_controller.hpp"

/** @brief Switch to offboard mode after minimum required setpoints */
void UAVController::timer_callback()
{
    // @note PX4 requires at least 10 setpoints before accepting offboard mode switch
    if (this->offboard_setpoint_counter_ == 10)
    {
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }

    this->publish_offboard_control_mode();

    if (this->offboard_setpoint_counter_ < 11)
    {
        this->publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
        this->offboard_setpoint_counter_++;
    }
}
