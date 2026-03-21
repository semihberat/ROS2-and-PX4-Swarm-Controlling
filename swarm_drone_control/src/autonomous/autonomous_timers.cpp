#include "autonomous.hpp"

// Mission state machine - called every 100ms
void SwarmMemberPathPlanner::state_cycle_callback()
{
    if (!current_neighbors_info_ || !current_wp_)
        return;

    if (this->latest_qr_info_)
    {
        this->current_mission = Mission::DO_PROCESS;
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
