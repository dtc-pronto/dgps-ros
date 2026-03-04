

#include "ros/dgps_node.hpp"

using namespace dgps;

DGPSNode::DGPSNode() : Node("dgps_node") 
{
    declare_parameter<std::string>("dev", "/dev/ttyACM0");
    declare_parameter<int>("baud", 460800);
    
    std::string dev = get_parameter("dev").as_string();
    int baud = get_parameter("baud").as_int();

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/dgps/fix", 10);
    heading_pub_ =create_publisher<geometry_msgs::msg::QuaternionStamped>("/dgps/heading", 10);
   
    rtcm_sub_ = create_subscription<rtcm_msgs::msg::Message>("/rtcm", 10, std::bind(&DGPSNode::rtcmCallback, this, std::placeholders::_1));

    dgps_ = std::make_unique<DifferentialGPS>(dev, baud);

    dgps_->setGpsCallback([this](dgps::NavSatFix nmea) { this->publishGPS(nmea); });
    dgps_->setAttitudeCallback([this](dgps::Vector3 attitude) { this->publishHeading(attitude); });

    dgps_->start();
}

void DGPSNode::rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg)
{
    if (!dgps_ || msg->message.empty()) return;
    dgps_->write(msg->message);
}

void DGPSNode::publishGPS(dgps::NavSatFix nmea)
{
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = now();
    msg.header.frame_id = "enu";

    msg.latitude = nmea.gps.latitude;
    msg.longitude = nmea.gps.longitude;
    msg.altitude = nmea.gps.altitude;

    msg.position_covariance[0] = nmea.variance.x;
    msg.position_covariance[4] = nmea.variance.y;
    msg.position_covariance[8] = nmea.variance.z;

    msg.status.status = (nmea.status > 0) ?
            sensor_msgs::msg::NavSatStatus::STATUS_FIX :
            sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    msg.position_covariance_type=sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    fix_pub_->publish(msg);
}

void DGPSNode::publishHeading(dgps::Vector3 attitude)
{
    geometry_msgs::msg::QuaternionStamped q;
    q.header.stamp = now();
    q.header.frame_id = "enu";

    q.quaternion.x = 0.0;
    q.quaternion.y = 0.0;
    q.quaternion.z = std::sin(attitude.z * 0.5);
    q.quaternion.w = std::cos(attitude.z * 0.5);

    heading_pub_->publish(q);
}
