#include "communication.hpp"

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NeighborsListener>());
    rclcpp::shutdown();
    return 0;
}