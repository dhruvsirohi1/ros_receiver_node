#include "aeye_ros2_driver/aeye_driver_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<aeye_ros2_driver::AeyeDriverNode>();
    // spin() keeps the node alive for parameter services, logging, etc.
    // Actual data flow is driven by the receiver thread, not the executor.
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
