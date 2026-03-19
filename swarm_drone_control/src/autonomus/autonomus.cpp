#include "autonomus.hpp"

// Lifecycle-managed autonomous path planner for swarm member drones

SwarmMemberPathPlanner::SwarmMemberPathPlanner() : LifecycleNode("swarm_member_path_planner")
{
    this->declare_parameter("sys_id", 1);
    this->declare_parameter("total_drones", 3);
    this->all_positions.reserve(20);
}

void SwarmMemberPathPlanner::setup_publishers_and_subscribers()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    std::string neighbors_topic = "/px4_" + std::to_string(sys_id_) + "/neighbors_info";
    this->neighbors_info_subscription_ = this->create_subscription<NeighborsInfo>(
        neighbors_topic, qos,
        std::bind(&SwarmMemberPathPlanner::neighbors_info_subscriber, this, std::placeholders::_1));

    std::string attitude_topic = "/px4_" + std::to_string(sys_id_) + "/fmu/out/vehicle_attitude";
    this->vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        attitude_topic, qos,
        std::bind(&SwarmMemberPathPlanner::vehicle_attitude_subscriber, this, std::placeholders::_1));

    std::string trajectory_topic = "/px4_" + std::to_string(sys_id_) + "/fmu/in/trajectory_setpoint";
    this->trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(trajectory_topic, 10);

    // Multi-Subscribing Synchronization
    std::string in_target_topic = "/px4_" + std::to_string(sys_id_) + "/in_target";
    this->in_target_publisher_ = this->create_publisher<custom_interfaces::msg::InTarget>(in_target_topic, 10);

    setup_in_target_subscribers(qos);
}

// Set up dynamic multi-subscriptions for target synchronization
void SwarmMemberPathPlanner::setup_in_target_subscribers(const rclcpp::QoS &qos)
{
    for (int i = 1; i <= total_drones_; ++i)
    {
        if (i != sys_id_)
        {
            std::string sub_topic = "/px4_" + std::to_string(i) + "/in_target";
            auto sub = this->create_subscription<custom_interfaces::msg::InTarget>(
                sub_topic, qos,
                std::bind(&SwarmMemberPathPlanner::in_target_callback, this, std::placeholders::_1));
            this->in_target_subs_.push_back(sub);
        }
    }
}

void SwarmMemberPathPlanner::setup_timers()
{
    this->timer_ = this->create_wall_timer(
        100ms, std::bind(&SwarmMemberPathPlanner::state_cycle_callback, this));
    this->timer_->cancel();

    this->timer_2 = this->create_wall_timer(100ms, std::bind(&SwarmMemberPathPlanner::collision_avoidance, this));
    this->timer_2->cancel();
}

void SwarmMemberPathPlanner::reset_timers()
{
    if (this->timer_)
    {
        this->timer_->reset();
    }
    if (this->timer_2)
    {
        this->timer_2->reset();
    }
}

void SwarmMemberPathPlanner::clear_pointers()
{
    if (this->timer_)
    {
        this->timer_->cancel();
        this->timer_.reset();
    }
    if (this->timer_2)
    {
        this->timer_2->cancel();
        this->timer_2.reset();
    }
    this->trajectory_setpoint_publisher_.reset();
    this->neighbors_info_subscription_.reset();
    this->vehicle_attitude_subscription_.reset();
    
    if (this->in_target_publisher_) {
        this->in_target_publisher_.reset();
    }
    this->in_target_subs_.clear();
}

LifecycleCallbackReturn SwarmMemberPathPlanner::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "ON_CONFIGURE: Allocating resources and setting up topics.");

    this->sys_id_ = this->get_parameter("sys_id").as_int();
    this->total_drones_ = this->get_parameter("total_drones").as_int();

    this->drones_in_target_.assign(this->total_drones_ + 1, false);

    // Instantiate resources, allocate memory
    std::vector<VehicleGlobalPosition> test_waypoints;
    test_waypoints.push_back(VehicleGlobalPosition().set__lat(47.397986).set__lon(8.546056).set__alt(target_altitude_));
    test_waypoints.push_back(VehicleGlobalPosition().set__lat(47.398186).set__lon(8.546056).set__alt(target_altitude_));
    test_waypoints.push_back(VehicleGlobalPosition().set__lat(47.398186).set__lon(8.546256).set__alt(target_altitude_));
    this->waypoint_manager_.set_waypoints(test_waypoints);

    setup_publishers_and_subscribers();
    setup_timers();

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn SwarmMemberPathPlanner::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_ACTIVATE: Activating publishers and starting mission.");

    this->trajectory_setpoint_publisher_->on_activate();
    this->in_target_publisher_->on_activate();
    
    // Resume or restart mission state logic
    this->waypoint_manager_.reset(); // restart from beginning
    this->current_wp_ = waypoint_manager_.current();
    this->current_mission = Mission::FORMATIONAL_TAKEOFF;
    
    reset_timers();
    reset_in_target_status();

    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn SwarmMemberPathPlanner::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_DEACTIVATE: Pausing mission execution and deactivating publishers.");

    this->timer_->cancel();
    this->timer_2->cancel();
    this->trajectory_setpoint_publisher_->on_deactivate();
    this->in_target_publisher_->on_deactivate();

    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn SwarmMemberPathPlanner::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_CLEANUP");
    clear_pointers();
    rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn SwarmMemberPathPlanner::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_SHUTDOWN");
    clear_pointers();
    rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn SwarmMemberPathPlanner::on_error(const rclcpp_lifecycle::State &previous_state)
{
    LOG_ERROR(this->get_logger(), "ON_ERROR");
    clear_pointers();
    rclcpp_lifecycle::LifecycleNode::on_error(previous_state);
    return LifecycleCallbackReturn::FAILURE;
}

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwarmMemberPathPlanner>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
