#include "autonomus.hpp"

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto node = std::make_shared<SwarmMemberPathPlanner>();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmMemberPathPlanner>()->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}