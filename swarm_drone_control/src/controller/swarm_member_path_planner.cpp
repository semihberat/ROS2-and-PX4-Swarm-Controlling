#include "swarm_drone_control/swarm_member_path_planner.hpp"

// === CONSTRUCTOR ===
SwarmMemberPathPlanner::SwarmMemberPathPlanner() : LifecycleNode("swarm_member_path_planner")
{
    // Initialize QoS profile for sensor data
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Declare parameter for each swarm member's system ID
    this->declare_parameter("sys_id", 1);

    std::string ntpc = "/px4_" + std::to_string(this->get_parameter("sys_id").as_int()) + "/neighbors_info";

    neighbors_info_subscription_ = this->create_subscription<NeighborsInfo>(
        ntpc, qos,
        std::bind(&SwarmMemberPathPlanner::path_planner_callback, this, _1));
}

// === TIMER CALLBACK ===
void SwarmMemberPathPlanner::timer_callback()
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        publish_trajectory_setpoint(target_vlat, target_vlon, 0.0, 0.0);
    }
}

LifecycleCallbackReturn SwarmMemberPathPlanner::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "ON_CONFIGURE");

    // Create lifecycle publisher
    std::string TRAJECTORY_SETPOINT = "/px4_" + std::to_string(this->get_parameter("sys_id").as_int()) + "/fmu/in/trajectory_setpoint";
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(TRAJECTORY_SETPOINT, 10);
    all_positions.reserve(20); // Örnek: maksimum 20 komşu için yer ayır
    // Create timer (cancelled until activation)
    timer_ = this->create_wall_timer(
        100ms, std::bind(&SwarmMemberPathPlanner::timer_callback, this));

    timer_->cancel();

    return LifecycleCallbackReturn::SUCCESS;
}

// Path Planner Callback Function
void SwarmMemberPathPlanner::path_planner_callback(const NeighborsInfo::SharedPtr msg)
{
    if (verification_count < verification_count_max)
    {
        verification_count++;
        all_positions = msg->neighbor_positions;
        all_positions.push_back(msg->main_position);

        auto center_of_gravity = CalculateCenterofGravity::calculate_cog<VehicleVerticalPositions, VehicleGlobalPosition>(all_positions);
        auto offsets = CalculateOffsetsFromCenter::calculate_offsets(center_of_gravity, offset_lat, offset_lon, all_positions.size());
        auto matched_position = offsets[msg->main_id - 1];

        target_position_ = matched_position;
    }

    VectoralDistance distance_left = CalculateDistance().calculate_distance<VectoralDistance>(
        msg->main_position.lat, msg->main_position.lon,
        target_position_.lat, target_position_.lon);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_vlat = distance_left.dlat_meter;
        target_vlon = distance_left.dlon_meter;
    }
    for (const auto &neighbor : msg->neighbor_positions)
    {
        auto uav_distance = CalculateDistance().calculate_distance<VectoralDistance>(
            msg->main_position.lat, msg->main_position.lon,
            neighbor.lat, neighbor.lon);

        if (uav_distance.distance <= collision_tolerance_m)
        {
            target_vlon = -uav_distance.dlon_meter + target_vlon;
            target_vlat = -uav_distance.dlat_meter + target_vlat;
        }
    }
}

void SwarmMemberPathPlanner::publish_trajectory_setpoint(float x, float y, float z, float yaw_rad)
{
    TrajectorySetpoint msg{};
    msg.velocity = {x, y, z};
    msg.position = {NAN, NAN, NAN};
    msg.yaw = NAN; // [-PI:PI]
    msg.yawspeed = yaw_rad;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

LifecycleCallbackReturn SwarmMemberPathPlanner::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_ACTIVATE");

    // Activate lifecycle publisher

    trajectory_setpoint_publisher_->on_activate();
    verification_count = 0;
    // Start timer
    timer_->reset();

    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}
LifecycleCallbackReturn SwarmMemberPathPlanner::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_DEACTIVATE");

    // Stop timer
    timer_->cancel();

    // Deactivate lifecycle publisher

    trajectory_setpoint_publisher_->on_deactivate();

    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}
LifecycleCallbackReturn SwarmMemberPathPlanner::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_CLEANUP");

    // Clean up lifecycle-managed resources
    timer_.reset();
    trajectory_setpoint_publisher_.reset();

    rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}
LifecycleCallbackReturn SwarmMemberPathPlanner::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "ON_SHUTDOWN");

    // Clean up resources on shutdown
    if (timer_)
    {
        timer_->cancel();
        timer_.reset();
    }
    trajectory_setpoint_publisher_.reset();

    rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}
LifecycleCallbackReturn SwarmMemberPathPlanner::on_error(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_ERROR(this->get_logger(), "ON_ERROR");

    // Clean up resources on error
    if (timer_)
    {
        timer_->cancel();
        timer_.reset();
    }
    trajectory_setpoint_publisher_.reset();

    rclcpp_lifecycle::LifecycleNode::on_error(previous_state);
    return LifecycleCallbackReturn::FAILURE;
}

int main(int argc, char *argv[])
{
    std::cout << "============== Path Planer ==============" << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmMemberPathPlanner>()->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
// 31 10 2025