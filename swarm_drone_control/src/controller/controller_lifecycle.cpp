#include "controller.hpp"

using namespace std::chrono_literals;

// Setup all publishers, subscribers for gamepad control
LifecycleCallbackReturn GamepadController::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "ON_CONFIGURE");

    int sys_id = this->get_parameter("sys_id").as_int();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    std::string OFFBOARD_CONTROL_MODE = "/px4_" + std::to_string(sys_id) + "/fmu/in/offboard_control_mode";
    std::string TRAJECTORY_SETPOINT = "/px4_" + std::to_string(sys_id) + "/fmu/in/trajectory_setpoint";
    std::string VEHICLE_COMMAND = "/px4_" + std::to_string(sys_id) + "/fmu/in/vehicle_command";
    std::string VEHICLE_ATTITUDE = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_attitude";
    std::string VEHICLE_STATUS = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_status_v1";

    this->offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(OFFBOARD_CONTROL_MODE, 10);
    this->trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(TRAJECTORY_SETPOINT, 10);
    this->vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(VEHICLE_COMMAND, 10);

    this->joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&GamepadController::joystick_callback, this, _1));
    this->vehicle_attitude_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        VEHICLE_ATTITUDE, qos, std::bind(&GamepadController::altitude_callback, this, _1));
    this->vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        VEHICLE_STATUS, qos, std::bind(&GamepadController::vehicle_status_callback, this, _1));

    this->offboard_setpoint_counter_ = 0;

    this->timer_ = this->create_wall_timer(100ms, std::bind(&GamepadController::controller_callback, this));
    this->timer_->cancel();

    return LifecycleCallbackReturn::SUCCESS;
}

// Activate publishers and start control loop
LifecycleCallbackReturn GamepadController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_ACTIVATE");

    this->offboard_control_mode_publisher_->on_activate();
    this->trajectory_setpoint_publisher_->on_activate();
    this->vehicle_command_publisher_->on_activate();

    this->timer_->reset();

    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

// Stop control and deactivate all publishers
LifecycleCallbackReturn GamepadController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_DEACTIVATE");

    this->offboard_control_mode_publisher_->on_deactivate();
    this->trajectory_setpoint_publisher_->on_deactivate();
    this->vehicle_command_publisher_->on_deactivate();

    this->timer_->cancel();

    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GamepadController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_CLEANUP");

    this->offboard_control_mode_publisher_.reset();
    this->trajectory_setpoint_publisher_.reset();
    this->vehicle_command_publisher_.reset();
    this->joy_subscriber_.reset();
    this->vehicle_attitude_subscriber_.reset();
    this->vehicle_status_subscriber_.reset();

    if (this->timer_)
    {
        this->timer_->cancel();
        this->timer_.reset();
    }

    rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GamepadController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_SHUTDOWN");

    this->offboard_control_mode_publisher_.reset();
    this->trajectory_setpoint_publisher_.reset();
    this->vehicle_command_publisher_.reset();
    this->joy_subscriber_.reset();
    this->vehicle_attitude_subscriber_.reset();
    this->vehicle_status_subscriber_.reset();

    if (this->timer_)
    {
        this->timer_->cancel();
        this->timer_.reset();
    }

    rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn GamepadController::on_error(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_ERROR(this->get_logger(), "ON_ERROR");

    // Clean up all resources on error
    if (this->timer_)
    {
        this->timer_->cancel();
        this->timer_.reset();
    }
    this->offboard_control_mode_publisher_.reset();
    this->trajectory_setpoint_publisher_.reset();
    this->vehicle_command_publisher_.reset();
    this->joy_subscriber_.reset();
    this->vehicle_attitude_subscriber_.reset();
    this->vehicle_status_subscriber_.reset();

    rclcpp_lifecycle::LifecycleNode::on_error(previous_state);
    return LifecycleCallbackReturn::FAILURE;
}
