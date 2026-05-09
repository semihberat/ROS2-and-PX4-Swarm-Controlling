
#include "drone_core/drone_core.hpp"

// Manual Composition
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto drone_core = std::make_shared<DroneCore>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(drone_core);
    executor.spin();
    rclcpp::shutdown();
}