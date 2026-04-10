#include "drone_core/drone_core.hpp"
#include "calculations/survey_plan.hpp"

rclcpp_action::GoalResponse DroneCore::goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveDrone::Goal> goal)
{
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received a new goal!");

    // Policy: preempt existing goal when receiving a new valid goal
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (goal_handle_)
        {
            if (goal_handle_->is_active())
            {
                RCLCPP_INFO(this->get_logger(), "Abort current goal and accept new goal");
                preempted_goal_id_ = goal_handle_->get_goal_id();
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Accepting the goal!");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DroneCore::cancel_callback(const std::shared_ptr<MoveDroneGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request!");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DroneCore::handle_accepted(const std::shared_ptr<MoveDroneGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal!");
    // Unutma

    execute_goal(goal_handle);
}

void DroneCore::execute_goal(const std::shared_ptr<MoveDroneGoalHandle> goal_handle)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }

    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveDrone::Result>();
    auto feedback = std::make_shared<MoveDrone::Feedback>();

    std::vector<custom_interfaces::msg::GeoPoint> points;
    points = goal->geo_points;
    if (goal->command == MoveDrone::Goal::GRID)
    {
        points = geo::GeoSurveyPlanner().generateGeoPath(goal->geo_points, service_parameters_.formation_radius);
    }

    size_t num_points = points.size();
    size_t i_p = 0;

    rclcpp::Rate loop_rate(100ms);

    while (rclcpp::ok())
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle->get_goal_id() == preempted_goal_id_)
            {
                result->message = "[PREEMPTION] Goal preempted by a new goal";
                result->success = false;
                goal_handle->abort(result);
                return;
            }
        }

        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->message = "[CANCELLATION] Goal was canceled";
            goal_handle->canceled(result);
            return;
        }

        if (goal->command == MoveDrone::Goal::HOLD)
        {
            this->publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
            result->success = true;
            result->message = "[HOLD] Command executed successfully";
            goal_handle->succeed(result);
        }
        else if (goal->command == MoveDrone::Goal::TAKEOFF)
        {
            if (!is_armed_)
                this->arm();

            float alt_diff = service_parameters_.takeoff_alt - drone_info_->geo_point.alt;
            float t_alt_vel = std::clamp(alt_diff, -service_parameters_.alt_vel, service_parameters_.alt_vel);

            if (alt_diff < 0.0f)
            {
                this->publish_trajectory_setpoint(0.0, 0.0, t_alt_vel, 0.0);
            }
            else
            {
                this->publish_trajectory_setpoint(0.0, 0.0, -t_alt_vel, 0.0);
            }

            if (alt_diff < service_parameters_.alt_vel)
            {
                this->publish_trajectory_setpoint(0.0, 0.0, -alt_diff * P_GAIN, 0.0);
                if (alt_diff < 0.25f)
                {
                    this->publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
                    result->success = true;
                    result->message = "[TAKEOFF] Command executed successfully";
                    goal_handle->succeed(result);
                    return;
                }
            }
        }

        else if (goal->command == MoveDrone::Goal::WAYPOINT || goal->command == MoveDrone::Goal::GRID)
        {
            if (!is_armed_)
            {
                result->success = false;
                result->message = "[WAYPOINT] Drone is not armed";
                goal_handle->abort(result);
                return;
            }
            // Minus is overloaded
            geo::Distance dist = points[i_p] - *this;
            double bearing = geo::calculate_bearing(drone_info_->geo_point, points[i_p]);
            float lat_vel = service_parameters_.velocity * cos(bearing);
            float lon_vel = service_parameters_.velocity * sin(bearing);

            double diff = bearing - drone_info_->yaw;

            if (diff > M_PI)
                diff -= 2.0 * M_PI;
            else if (diff < -M_PI)
                diff += 2.0 * M_PI;
            float yaw_vel = service_parameters_.yaw_vel * diff;

            this->publish_trajectory_setpoint(lat_vel, lon_vel, 0.0, yaw_vel);
            if (dist.d < service_parameters_.velocity / 2)
            {
                this->publish_trajectory_setpoint(dist.d_lat * P_GAIN, dist.d_lon * P_GAIN, 0.0, yaw_vel * P_GAIN);
                if (dist.d < 0.25)
                {
                    this->publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0);
                    i_p++;
                    if (i_p >= num_points)
                    {
                        result->success = true;
                        result->message = "[WAYPOINT] Command executed successfully";
                        goal_handle->succeed(result);
                        return;
                    }
                }
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown command received: %d", goal->command);
            result->success = false;
            result->message = "[UNKNOWN] Unknown command received";
            goal_handle->abort(result);
            return;
        }

        feedback->current.geo_point = drone_info_->geo_point;
        feedback->current.id = drone_info_->id;
        loop_rate.sleep();
    }
}