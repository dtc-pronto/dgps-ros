#include "ros/septentrio_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions opts;
    auto node = std::make_shared<dgps::SeptentrioNode>(opts);
    RCLCPP_INFO(node->get_logger(), "SeptentrioNode spinning...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
