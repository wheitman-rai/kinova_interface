#include <rclcpp/rclcpp.hpp>
#include "kinova_interface/kinova_interface.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<kinova_interface::KinovaInterfaceNode>();

    // Use a multithreaded executor for better responsiveness
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}