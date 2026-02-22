#include "controller.hpp"

// Lifecycle-managed gamepad controller for manual drone control
GamepadController::GamepadController() : LifecycleNode("gamepad_controller")
{
    this->declare_parameter("sys_id", 1);
}
