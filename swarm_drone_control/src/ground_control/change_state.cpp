#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace lifecycle_msgs::msg;
using namespace lifecycle_msgs::srv;
using namespace sensor_msgs::msg;

class LifecycleNodeManager : public rclcpp::Node
{
public:
    LifecycleNodeManager() : Node("lifecycle_node_manager")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        this->declare_parameter("sys_id", 1);
        int sys_id = this->get_parameter("sys_id").as_int();
        
        std::string path_planner_service = "/path_planner_" + std::to_string(sys_id) + "/change_state";
        std::string gamepad_controller_service = "/gamepad_controller_" + std::to_string(sys_id) + "/change_state";

        joy_listener = this->create_subscription<Joy>("/joy",
                                                      qos,
                                                      std::bind(&LifecycleNodeManager::joy_callback, this, _1));

        client_path_planner_ = this->create_client<ChangeState>(path_planner_service);
        client_gamepad_ = this->create_client<ChangeState>(gamepad_controller_service);
    }

    void joy_callback(const Joy::SharedPtr msg)
    {
        if (msg->buttons[0] == 1) // Assuming button 0 is for 'configure'
        {
            change_state(client_path_planner_, "configure", Transition::TRANSITION_CONFIGURE);
            change_state(client_path_planner_, "activate", Transition::TRANSITION_ACTIVATE);
            
            change_state(client_gamepad_, "deactivate", Transition::TRANSITION_DEACTIVATE);
            change_state(client_gamepad_, "cleanup", Transition::TRANSITION_CLEANUP);
        }
        else if (msg->buttons[1] == 1) // Assuming button 1 is for 'activate'
        {
            change_state(client_path_planner_, "deactivate", Transition::TRANSITION_DEACTIVATE);
            change_state(client_path_planner_, "cleanup", Transition::TRANSITION_CLEANUP);

            change_state(client_gamepad_, "configure", Transition::TRANSITION_DEACTIVATE);
            change_state(client_gamepad_, "activate", Transition::TRANSITION_CLEANUP);
        }
    }

    void change_state(rclcpp::Client<ChangeState>::SharedPtr client, std::string label, int transition_id)
    {
        if (!client->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
            return;
        }

        auto request = std::make_shared<ChangeState::Request>();
        request->transition.id = transition_id;
        request->transition.label = label;
        
        RCLCPP_INFO(this->get_logger(), "Transitioning %s to %s", client->get_service_name(), label.c_str());
        auto future = client->async_send_request(request);
    }

private:
    rclcpp::Client<ChangeState>::SharedPtr client_path_planner_;
    rclcpp::Client<ChangeState>::SharedPtr client_gamepad_;
    rclcpp::Subscription<Joy>::SharedPtr joy_listener;

};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleNodeManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}