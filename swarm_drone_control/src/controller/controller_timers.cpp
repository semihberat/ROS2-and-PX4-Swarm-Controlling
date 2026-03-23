#include "controller.hpp"

// Process joystick input and send control commands
void GamepadController::controller_callback()
{
    // testing rclcpp info for timer is working?

    if (this->joystick_state_ == nullptr || this->vehicle_status_ == nullptr || this->vehicle_attitude_ == nullptr)
        return;

    // Arm drone when left joystick held at bottom-right
    if (this->joystick_state_->axes[0] < -0.5f && this->joystick_state_->axes[1] < -0.5f && this->vehicle_status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
    {

        arm_timer++;
        RCLCPP_INFO(this->get_logger(), "[ARM] Holding: %d/20, axes[0]=%.2f, axes[1]=%.2f",
                    arm_timer, this->joystick_state_->axes[0], this->joystick_state_->axes[1]);
        if (arm_timer > 20)
        {
            RCLCPP_WARN(this->get_logger(), "*** SENDING ARM COMMAND ***");
            this->arm();
            arm_timer = 0;
        }
    }

    this->relative_movement(
        (this->joystick_state_->axes[4] + this->joystick_state_->axes[7]) * 10,
        (-this->joystick_state_->axes[3] + this->joystick_state_->axes[6]) * 10,
        -this->joystick_state_->axes[1] * 10,
        -this->joystick_state_->axes[0] * 3.14f);
}