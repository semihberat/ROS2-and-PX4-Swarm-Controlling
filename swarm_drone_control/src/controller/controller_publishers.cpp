#include "controller.hpp"
#include "calculations/geographic.hpp"

/** @brief Send arming command to drone */
void GamepadController::arm()
{
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/** @brief Send disarming command to drone */
void GamepadController::disarm()
{
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Convert joystick commands to world frame velocities
 * @param velocity_x Body-frame forward velocity
 * @param velocity_y Body-frame lateral velocity
 * @param velocity_z Vertical velocity
 * @param yawspeed Yaw rate
 */
void GamepadController::relative_movement(float velocity_x, float velocity_y, float velocity_z, float yawspeed)
{
    // @note Extract yaw from quaternion using atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2))
    float yaw_angle = atan2f(2.0f * (this->vehicle_attitude_->q[0] * this->vehicle_attitude_->q[3] + this->vehicle_attitude_->q[1] * this->vehicle_attitude_->q[2]),
                             1.0f - 2.0f * (this->vehicle_attitude_->q[2] * this->vehicle_attitude_->q[2] + this->vehicle_attitude_->q[3] * this->vehicle_attitude_->q[3]));

    // Transform body-frame velocities to NED world frame based on the swarm's moving forward direction
    float velocity_north = velocity_x * cosf(yaw_angle) - velocity_y * sinf(yaw_angle);
    float velocity_east = velocity_x * sinf(yaw_angle) + velocity_y * cosf(yaw_angle);

    // Apply Rigid Body swarm formation calculation if neighbor info exists
    if (this->current_neighbors_info_ != nullptr)
    {
        std::vector<px4_msgs::msg::VehicleGlobalPosition> all_pos = this->current_neighbors_info_->neighbor_positions;
        all_pos.push_back(this->current_neighbors_info_->main_position);

        px4_msgs::msg::VehicleGlobalPosition cog = geo::calculate_cog<px4_msgs::msg::VehicleGlobalPosition>(all_pos);

        DLatDLon offset_from_cog = geo::calculate_distance<DLatDLon>(cog.lat, cog.lon,
                                                                     this->current_neighbors_info_->main_position.lat,
                                                                     this->current_neighbors_info_->main_position.lon);

        double bearing_from_cog = geo::calculate_bearing(cog, this->current_neighbors_info_->main_position);

        // Record initial shape
        if (!this->initial_distance_set_)
        {
            this->target_distance_to_cog_ = offset_from_cog.distance;
            this->initial_distance_set_ = true;
        }

        // Rigid body circular motion:  V_tangent = omega x r  (omega is yawspeed from joystick)
        // Tangent direction is bearing_from_cog + PI/2
        double tangential_velocity = yawspeed * offset_from_cog.distance;

        // Note: Tangential components to add to North and East target velocities
        velocity_north += tangential_velocity * std::cos(bearing_from_cog + geo::PI_VAL / 2.0);
        velocity_east += tangential_velocity * std::sin(bearing_from_cog + geo::PI_VAL / 2.0);

        // --- Simple Cohesion (Formation Keeper) ---
        // Push the drone back towards its initial radius from the CoG to prevent drifting away/shrinking
        double cohesion_gain = 0.8;
        double target_radius_error = this->target_distance_to_cog_ - offset_from_cog.distance;

        // If error > 0, we need to push OUT (along bearing_from_cog). If error < 0, pull IN.
        velocity_north += target_radius_error * std::cos(bearing_from_cog) * cohesion_gain;
        velocity_east += target_radius_error * std::sin(bearing_from_cog) * cohesion_gain;

        // --- Simple Collision Avoidance ---
        double avoidance_radius = 2.0; // Avoidance trigger distance in meters
        double repel_gain = 2.0;

        for (const auto &neighbor_pos : this->current_neighbors_info_->neighbor_positions)
        {
            DLatDLon dist_to_neighbor = geo::calculate_distance<DLatDLon>(
                this->current_neighbors_info_->main_position.lat,
                this->current_neighbors_info_->main_position.lon,
                neighbor_pos.lat,
                neighbor_pos.lon);

            if (dist_to_neighbor.distance > 0.01 && dist_to_neighbor.distance < avoidance_radius)
            {
                double repulsion_force = repel_gain * (avoidance_radius - dist_to_neighbor.distance);
                double bearing_from_neighbor = geo::calculate_bearing(neighbor_pos, this->current_neighbors_info_->main_position);

                velocity_north += repulsion_force * std::cos(bearing_from_neighbor);
                velocity_east += repulsion_force * std::sin(bearing_from_neighbor);
            }
        }
    }

    this->publish_trajectory_setpoint(velocity_north, velocity_east, velocity_z, yawspeed);
}

void GamepadController::publish_trajectory_setpoint(float x, float y, float z, float yawspeed)
{
    TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {x, y, z};
    msg.yaw = NAN;
    msg.yawspeed = yawspeed;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->trajectory_setpoint_publisher_->publish(msg);
}

void GamepadController::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = this->get_parameter("sys_id").as_int() + 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->vehicle_command_publisher_->publish(msg);
}
