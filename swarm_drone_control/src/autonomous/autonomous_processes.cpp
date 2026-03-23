#include "autonomous.hpp"

// Execute mission-specific task at target location
void SwarmMemberPathPlanner::do_process()
{
    switch (this->current_process)
    {
    case DoProcess::FORMATION:
        if (this->latest_qr_info_->gorev.formasyon.aktif)
            this->formation(this->latest_qr_info_->gorev.formasyon);
        else
            this->current_process = DoProcess::MANUEVER_PITCH_ROLL;
        break;
    case DoProcess::MANUEVER_PITCH_ROLL:
        if (this->latest_qr_info_->gorev.manevra_pitch_roll.aktif)
            this->manuever_pitch_roll(this->latest_qr_info_->gorev.manevra_pitch_roll);
        else
            this->current_process = DoProcess::ALTITUDE_CHANGE;
        // Implement a specific pitch/roll manuever (e.g., quick left-right)
        break;
    case DoProcess::ALTITUDE_CHANGE:
        if (this->latest_qr_info_->gorev.irtifa_degisim.aktif)
            this->altitude_change(this->latest_qr_info_->gorev.irtifa_degisim);
        else
            this->current_process = DoProcess::LEAVE_THE_SWARM;
        // Perform a quick altitude change (e.g., up and down)
        break;
    case DoProcess::LEAVE_THE_SWARM:
        if (this->latest_qr_info_->suruden_ayrilma.aktif)
            this->leave_the_swarm(this->latest_qr_info_->suruden_ayrilma);
        else
            this->current_process = DoProcess::NEXT;
        // Move in a specific direction to "leave" the swarm formation
        break;

    case DoProcess::NEXT:
        if (this->latest_qr_info_->qr_id == 0)
        {
            LOG_MISSION(this->get_logger(), "[MISSION INFO] No more processes. Mission complete!");
            end_task();
        }
        else
        {
            LOG_MISSION(this->get_logger(), "[MISSION INFO] Moving to next process...");
            next_step(Mission::FORMATIONAL_ROTATION);
        }
        break;
    default:
        LOG_ERROR(this->get_logger(), "[MISSION ERROR] Unknown process state! NEXT!.");
        this->current_process = DoProcess::NEXT;
        break;
    }
}

void SwarmMemberPathPlanner::formation(const custom_interfaces::msg::Formation &formasyon) {}
void SwarmMemberPathPlanner::manuever_pitch_roll(const custom_interfaces::msg::PitchRollMovement &manevra) {}
void SwarmMemberPathPlanner::altitude_change(const custom_interfaces::msg::AltitudeChange &irtifa_degisim) {}
void SwarmMemberPathPlanner::leave_the_swarm(const custom_interfaces::msg::LeaveTheHerd &suruden_ayrilma) {}
