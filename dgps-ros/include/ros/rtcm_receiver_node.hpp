/*!
* @Date 2026
*
* @About ROS2 component that receives raw RTCM corrections from the base
* station over a ZeroMQ PUB/SUB stream and republishes them on a ROS topic
* (default /rtcm) for the Septentrio / DGPS drivers to forward to the receiver.
*
* Mirrors the base station's rtk_correction broadcaster: the broadcaster reads
* the base GPS with pyrtcm and PUBs raw RTCM bytes over tcp://ip:port; this node
* is the rover-side SUB that turns those packets back into rtcm_msgs/Message.
*/
#pragma once

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rtcm_msgs/msg/message.hpp>

namespace dgps
{
class RtcmReceiverNode : public rclcpp::Node
{
    public:
        explicit RtcmReceiverNode(const rclcpp::NodeOptions& options);
        ~RtcmReceiverNode() override;

    private:
        void receiveLoop();

        std::string ip_;
        int port_;
        std::string topic_;
        std::string frame_id_;
        int poll_timeout_ms_;
        std::string endpoint_;

        // libzmq handles kept as void* so zmq.h stays out of the public header.
        void* context_{nullptr};
        void* socket_{nullptr};

        std::atomic<bool> running_{false};
        std::thread recv_thread_;

        rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr rtcm_pub_;
};
} // namespace dgps
