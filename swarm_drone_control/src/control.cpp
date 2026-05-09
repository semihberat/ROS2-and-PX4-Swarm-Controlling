#include "drone_core/drone_core.hpp"

void DroneCore::takeoff(float altitude)
{
    if (!is_armed_)
        this->arm();

    altitude_controller_.setMaxOutput(service_parameters_.alt_vel);
    altitude_controller_.setSetpoint(-altitude);
    float z_vel = altitude_controller_.update(this->drone_info_->geo_point.alt);
    this->publish_trajectory_setpoint(0.0f, 0.0f, z_vel, 0.0f);
}

void DroneCore::land()
{
    publish_trajectory_setpoint(0.0f, 0.0f, 1.0f, 0.0f);
    if (land_detected_)
    {
        this->disarm();
        if (!is_armed_)
        {
            this->req_->command = custom_interfaces::srv::DroneCommands::Request::HOLD;
            return;
        }
    }
}

void DroneCore::hold()
{
    if (this->is_armed_ && this->land_detected_)
    {
        this->disarm();
    }
    else
    {
        publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
    }
}

void DroneCore::goto_waypoints(const std::vector<custom_interfaces::msg::GeoPoint> &waypoints)
{
    geo::Distance dist = waypoints[i_p] - *this;
    double bearing = geo::calculate_bearing(this->drone_info_->geo_point, waypoints[i_p]);
    horizontal_PID_.setMaxOutput(service_parameters_.velocity);

    horizontal_PID_.setSetpoint(0.0);

    float vel = horizontal_PID_.update(-dist.d);
    float lat_vel = vel * std::cos(bearing);
    float lon_vel = vel * std::sin(bearing);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        lat_vel -= forces_.vlat;
        lon_vel -= forces_.vlon;
    }
    // Print lat velocity and lon velocity
    RCLCPP_INFO(this->get_logger(), "Lat Vel: %f, Lon Vel: %f", lat_vel, lon_vel);
    double diff = bearing - this->drone_info_->yaw;
    if (diff > M_PI)
        diff -= 2 * M_PI;
    else if (diff < -M_PI)
        diff += 2 * M_PI;
    float yaw_vel = diff * service_parameters_.yaw_vel;
    this->publish_trajectory_setpoint(lat_vel, lon_vel, 0.0f, yaw_vel);
    if (dist.d <= 0.25)
    {

        if (i_p >= waypoints.size())
        {
            this->req_->command = custom_interfaces::srv::DroneCommands::Request::HOLD;
            return;
        }
    }
}

void DroneCore::collision_avoidance()
{
    using namespace custom_interfaces::srv;
    std::lock_guard<std::mutex> lock(mutex_);
    forces_.vlat = 0.0;
    forces_.vlon = 0.0;

    for (const auto &neighbor : this->swarm_info_)
    {
        if (neighbor == nullptr || neighbor->id == this->sys_id)
            continue;

        geo::Distance dist = neighbor->geo_point - *this;

        if (this->req_->command == DroneCommands::Request::TAKEOFF ||
            this->req_->command == DroneCommands::Request::HOLD)
        {
            init_dists_[neighbor->id - 1] = dist;
            continue;
        }

        swarm_PIDs_[neighbor->id - 1].setMaxOutput(1.0);
        swarm_PIDs_[neighbor->id - 1].setSetpoint(0.0);
        // Calculate desired relative displacement vs current displacement
        double error_lat = init_dists_[neighbor->id - 1].d_lat - dist.d_lat;
        double error_lon = init_dists_[neighbor->id - 1].d_lon - dist.d_lon;

        // Use P-control logic to calculate restoration forces
        // Gain value should be tuned, using 1.0 as a starting point
        forces_.vlat += swarm_PIDs_[neighbor->id - 1].update(error_lat);
        forces_.vlon += swarm_PIDs_[neighbor->id - 1].update(error_lon);
    }
}