#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace sensor_msgs::msg;
using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace lifecycle_msgs::msg;
using namespace lifecycle_msgs::srv;

class LifecycleNodeManager: public rclcpp::Node
{
    public:
    LifecycleNodeManager() : Node("formation_control")
    {
       this->declare_parameter("sys_id", 1);
       int node_id = this->get_parameter("sys_id").as_int();
       std::string node_name = "/path_planner_" + std::to_string(node_id);
       std::string service_change_state_name = node_name + "/change_state";
       this->client_ = this->create_client<ChangeState>(service_change_state_name);
       this->gamepad_subscription_ = this->create_subscription<Joy>(
        "joy", 10, std::bind(&LifecycleNodeManager::joy_callback, this, _1));
    }

    void joy_callback(const Joy::SharedPtr joy_msg){
        RCLCPP_INFO(this->get_logger(), "JOY BUTTONS: %d", joy_msg->buttons[0]);
        switch(joy_msg->buttons[0]){
            case 1:
                change_state("configure", Transition::TRANSITION_CONFIGURE);
                rclcpp::sleep_for(500ms);
                change_state("activate", Transition::TRANSITION_ACTIVATE);
                break;
        }
    }
    
    void change_state(std::string label, int transition_id){
        while(!this->client_->service_is_ready()){
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
            rclcpp::sleep_for(1s);
        }
        auto request = std::make_shared<ChangeState::Request>();
        request->transition.id = transition_id;
        request->transition.label = label;
        current_state_ = label;
        
        RCLCPP_INFO(this->get_logger(), "CURRENT_STATE: %s", current_state_.c_str());

        auto future = client_->async_send_request(request);
    }

private:
    void response_callback(rclcpp::Client<ChangeState>::SharedFuture future){
        auto response = future.get();
    } 
    
    rclcpp::Client<ChangeState>::SharedPtr client_;
    rclcpp::Subscription<Joy>::SharedPtr gamepad_subscription_;
    std::string current_state_;
};
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleNodeManager>();

    rclcpp::spin(node);
    rclcpp::shutdown();

}