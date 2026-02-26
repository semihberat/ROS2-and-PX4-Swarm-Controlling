#include "autonomus.hpp"

// Mission state machine - called every 100ms
void SwarmMemberPathPlanner::state_cycle_callback()
{
    if (this->current_neighbors_info_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for neighbors info...");
        return;
    }

    switch (this->current_mission)
    {
    case Mission::FORMATIONAL_TAKEOFF:
        formational_takeoff();
        break;
    case Mission::FORMATIONAL_ROTATION:
        formational_rotation();
        break;
    case Mission::GOTO_POSITION:

        goto_position();
        break;
    case Mission::DO_PROCESS:
        do_process();
        break;
    case Mission::END_TASK:
        end_task();
        break;
    }
}

void SwarmMemberPathPlanner::collision_avoidance()
{
}

void SwarmMemberPathPlanner::collision_avoidance()
{
}