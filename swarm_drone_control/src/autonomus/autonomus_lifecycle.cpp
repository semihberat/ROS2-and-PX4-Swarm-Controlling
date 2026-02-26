#include "autonomus.hpp"

using namespace std::chrono_literals;

// Setup all publishers, subscribers and timers (but don't activate yet)
LifecycleCallbackReturn SwarmMemberPathPlanner::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "ON_CONFIGURE");

    int sys_id = this->get_parameter("sys_id").as_int();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    std::string neighbors_topic = "/px4_" + std::to_string(sys_id) + "/neighbors_info";
    this->neighbors_info_subscription_ = this->create_subscription<NeighborsInfo>(
        neighbors_topic, qos,
        std::bind(&SwarmMemberPathPlanner::neighbors_info_subscriber, this, std::placeholders::_1));

    std::string attitude_topic = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_attitude";
    this->vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        attitude_topic, qos,
        std::bind(&SwarmMemberPathPlanner::vehicle_attitude_subscriber, this, std::placeholders::_1));

    std::string trajectory_topic = "/px4_" + std::to_string(sys_id) + "/fmu/in/trajectory_setpoint";
    this->trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(trajectory_topic, 10);

    this->timer_ = this->create_wall_timer(
        100ms, std::bind(&SwarmMemberPathPlanner::state_cycle_callback, this));
    this->timer_->cancel();

    // this->timer_2 = this->create_wall_timer(100ms, std::bind(&SwarmMemberPathPlanner::state_cycle_callback, this));
    // this->timer_2->cancel();

    current_mission = Mission::FORMATIONAL_TAKEOFF;

    return LifecycleCallbackReturn::SUCCESS;
}

// Activate publishers and start mission execution
LifecycleCallbackReturn SwarmMemberPathPlanner::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_ACTIVATE");

    this->trajectory_setpoint_publisher_->on_activate();

    this->timer_->reset();
    // this->timer_2->reset();

    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

// Stop mission execution and deactivate publishers
LifecycleCallbackReturn SwarmMemberPathPlanner::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_DEACTIVATE");

    this->timer_->cancel();
    // this->timer_2->cancel();
    this->trajectory_setpoint_publisher_->on_deactivate();

    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

// Clean up all resources for reconfiguration
LifecycleCallbackReturn SwarmMemberPathPlanner::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_CLEANUP");

    if (this->timer_)
    {
        this->timer_->cancel();
        this->timer_.reset();
    }
    /*     if (this->timer_2)
        {
            this->timer_2->cancel();
            this->timer_2.reset();
        } */
    this->trajectory_setpoint_publisher_.reset();
    this->neighbors_info_subscription_.reset();
    this->vehicle_attitude_subscription_.reset();

    rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn SwarmMemberPathPlanner::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_SHUTDOWN");

    if (this->timer_)
    {
        this->timer_->cancel();
        this->timer_.reset();
    }
    /*  if (this->timer_2)
     {
         this->timer_2->cancel();
         this->timer_2.reset();
     } */
    this->trajectory_setpoint_publisher_.reset();
    this->neighbors_info_subscription_.reset();
    this->vehicle_attitude_subscription_.reset();

    rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn SwarmMemberPathPlanner::on_error(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_ERROR(this->get_logger(), "ON_ERROR");

    if (this->timer_)
    {
        this->timer_->cancel();
        this->timer_.reset();
    }
    /*     if (this->timer_2)
        {
            this->timer_2->cancel();
            this->timer_2.reset();
        } */
    this->trajectory_setpoint_publisher_.reset();
    this->neighbors_info_subscription_.reset();
    this->vehicle_attitude_subscription_.reset();

    rclcpp_lifecycle::LifecycleNode::on_error(previous_state);
    return LifecycleCallbackReturn::FAILURE;
}
