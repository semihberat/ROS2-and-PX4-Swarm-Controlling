#include "autonomous.hpp"

/**
 * @brief Send velocity commands to PX4
 * @param x North velocity (m/s)
 * @param y East velocity (m/s)
 * @param z Down velocity (m/s)
 * @param yaw_rad Yaw rate (rad/s)
 */
void SwarmMemberPathPlanner::publish_trajectory_setpoint(float x, float y, float z, float yaw_rad)
{
    TrajectorySetpoint msg{};
    msg.velocity = {x, y, z};
    msg.position = {NAN, NAN, NAN};
    msg.yaw = NAN;
    msg.yawspeed = yaw_rad;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->trajectory_setpoint_publisher_->publish(msg);
}
