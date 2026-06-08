/*!
* @Date 2026
*
* @About Implementation of the ZeroMQ -> /rtcm bridge. See header for context.
*/
#include "ros/rtcm_receiver_node.hpp"

#include <zmq.h>

using namespace dgps;

RtcmReceiverNode::RtcmReceiverNode(const rclcpp::NodeOptions& options)
    : Node("rtcm_receiver", options)
{
    // Defaults match the base station's rtk_correction broadcaster.launch
    // (ip 10.10.10.10, port 7505). The broadcaster binds; this node connects.
    ip_              = declare_parameter<std::string>("ip", "10.10.10.10");
    port_            = declare_parameter<int>("port", 7505);
    topic_           = declare_parameter<std::string>("rtcm_topic", "/rtcm");
    frame_id_        = declare_parameter<std::string>("frame_id", "");
    poll_timeout_ms_ = declare_parameter<int>("poll_timeout_ms", 100);

    endpoint_ = "tcp://" + ip_ + ":" + std::to_string(port_);

    rtcm_pub_ = create_publisher<rtcm_msgs::msg::Message>(topic_, rclcpp::QoS(10));

    context_ = zmq_ctx_new();
    socket_  = zmq_socket(context_, ZMQ_SUB);
    // Subscribe to every message (the broadcaster sends a single unfiltered stream).
    zmq_setsockopt(socket_, ZMQ_SUBSCRIBE, "", 0);

    // zmq_connect on a SUB socket succeeds even if the broadcaster is not up yet
    // (it connects lazily); a non-zero return means a malformed endpoint.
    if (zmq_connect(socket_, endpoint_.c_str()) != 0)
    {
        RCLCPP_FATAL(get_logger(), "[RTK] Failed to connect ZMQ SUB to %s: %s",
                     endpoint_.c_str(), zmq_strerror(zmq_errno()));
        throw std::runtime_error("zmq_connect failed");
    }

    RCLCPP_INFO(get_logger(),
                "[RTK] Listening for RTCM broadcast at %s, republishing on %s",
                endpoint_.c_str(), topic_.c_str());

    running_ = true;
    recv_thread_ = std::thread(&RtcmReceiverNode::receiveLoop, this);
}

RtcmReceiverNode::~RtcmReceiverNode()
{
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    if (socket_)  { zmq_close(socket_); socket_ = nullptr; }
    if (context_) { zmq_ctx_term(context_); context_ = nullptr; }
}

void RtcmReceiverNode::receiveLoop()
{
    uint64_t msg_count  = 0;
    uint64_t byte_count = 0;

    while (running_ && rclcpp::ok())
    {
        zmq_pollitem_t item{};
        item.socket = socket_;
        item.events = ZMQ_POLLIN;

        // Poll with a timeout so the thread wakes periodically to re-check
        // running_/rclcpp::ok() and can shut down promptly.
        int rc = zmq_poll(&item, 1, poll_timeout_ms_);
        if (rc < 0)
        {
            if (zmq_errno() == ETERM) break;  // context terminated
            continue;
        }
        if (rc == 0 || !(item.revents & ZMQ_POLLIN)) continue;  // timeout / nothing ready

        zmq_msg_t zmsg;
        zmq_msg_init(&zmsg);
        int n = zmq_msg_recv(&zmsg, socket_, 0);
        if (n < 0)
        {
            zmq_msg_close(&zmsg);
            if (zmq_errno() == ETERM) break;
            continue;
        }

        const auto* data = static_cast<const uint8_t*>(zmq_msg_data(&zmsg));
        size_t sz = zmq_msg_size(&zmsg);

        if (sz > 0)
        {
            rtcm_msgs::msg::Message msg;
            msg.header.stamp = now();
            msg.header.frame_id = frame_id_;
            msg.message.assign(data, data + sz);
            rtcm_pub_->publish(msg);

            if (msg_count == 0)
                RCLCPP_INFO(get_logger(),
                            "[RTK] First RTCM packet received from %s (%zu bytes)",
                            endpoint_.c_str(), sz);

            ++msg_count;
            byte_count += sz;

            // Heartbeat while corrections are flowing; silence here means the
            // base station stopped broadcasting or the link dropped.
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[RTK] Forwarding corrections: %lu packets / %lu bytes on %s",
                                 static_cast<unsigned long>(msg_count),
                                 static_cast<unsigned long>(byte_count),
                                 topic_.c_str());
        }

        zmq_msg_close(&zmsg);
    }

    RCLCPP_INFO(get_logger(), "[RTK] Receive loop stopped (%lu packets total)",
                static_cast<unsigned long>(msg_count));
}

RCLCPP_COMPONENTS_REGISTER_NODE(dgps::RtcmReceiverNode)
