#include "ros/rtcm_receiver_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions opts;
    auto node = std::make_shared<dgps::RtcmReceiverNode>(opts);
    RCLCPP_INFO(node->get_logger(), "RtcmReceiverNode spinning...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
