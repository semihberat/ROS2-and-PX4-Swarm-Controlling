#include "autonomus.hpp"

void SwarmMemberPathPlanner::in_target_callback(const InTarget::Request::SharedPtr request,
                                                const InTarget::Response::SharedPtr response)
{
    if (request->is_in_position)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        RCLCPP_INFO(this->get_logger(), "Received in_target request: Drone %d is in position", request->drone_id);
        positioned_drones_[request->drone_id - 1] = request;
        response->success = true;
        response->message = "Position received successfully.";
    }
    else
    {
        response->success = false;
        response->message = "Invalid request or same drone ID.";
    }
}

void SwarmMemberPathPlanner::call_in_target_client()
{
    if (!in_target_client_->wait_for_service(std::chrono::milliseconds(200)))
    {
        RCLCPP_WARN(this->get_logger(), "in_target service not online yet, skipping request...");
        return;
    }

    auto request = std::make_shared<InTarget::Request>();
    request->drone_id = this->get_parameter("sys_id").as_int();
    request->is_in_position = true;
    in_target_client_->async_send_request(request, std::bind(&SwarmMemberPathPlanner::in_target_client_callback, this, _1));
}

void SwarmMemberPathPlanner::in_target_client_callback(rclcpp::Client<InTarget>::SharedFuture future)
{
    auto result = future.get();
    if (result->success)
    {
        LOG_SUCCESS(this->get_logger(), "Successfully informed neighbors of being in position");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to inform neighbors of being in position: %s", result->message.c_str());
    }
}
