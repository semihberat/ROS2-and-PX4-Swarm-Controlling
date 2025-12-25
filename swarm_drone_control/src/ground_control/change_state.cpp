#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace lifecycle_msgs::msg;
using namespace lifecycle_msgs::srv;

class LifecycleNodeManager : public rclcpp::Node
{
public:
    LifecycleNodeManager() : Node("lifecycle_node_manager")
    {
        this->declare_parameter("sys_id", 1);
        int sys_id = this->get_parameter("sys_id").as_int();
        std::string path_planner = "/path_planner_" + std::to_string(sys_id);
        std::string service_change_state_name = path_planner + "/change_state";
        this->client_ = this->create_client<ChangeState>(service_change_state_name);
    }

    void change_state(std::string label, int transition_id)
    {
        while (!this->client_->service_is_ready())
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
            rclcpp::sleep_for(1s);
        }
        auto request = std::make_shared<ChangeState::Request>();
        request->transition.id = transition_id;
        request->transition.label = label;
        current_state_ = label;

        RCLCPP_INFO(this->get_logger(), "CURRENT_STATE: %s", current_state_.c_str());

        auto future = client_->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    }

private:
    rclcpp::Client<ChangeState>::SharedPtr client_;
    std::string current_state_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleNodeManager>();

    node->change_state("configure", Transition::TRANSITION_CONFIGURE);
    std::cout << "SLEEP 5 SECONDS..." << std::endl;
    sleep(1);
    node->change_state("activate", Transition::TRANSITION_ACTIVATE);

    rclcpp::spin(node);
    rclcpp::shutdown();
}