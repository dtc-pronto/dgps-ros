/*!
* @Date 2026
*
* @About ROS2 component implementation for the Septentrio mosaic-G5 P3H.
* Mirrors dgps_node topic layout, prefixed with /sept.
*/
#include <cmath>
#include <cstring>
#include <limits>

#include "ros/septentrio_node.hpp"

using namespace dgps;

SeptentrioNode::SeptentrioNode(const rclcpp::NodeOptions& options) : Node("septentrio_node", options)
{
    declare_parameter<std::string>("nmea_dev",  "/dev/ttyACM0");  // USB1
    declare_parameter<int>("nmea_baud", 115200);
    declare_parameter<std::string>("rtcm_dev",  "/dev/ttyACM1");  // USB2; set empty to disable RTCM
    declare_parameter<int>("rtcm_baud", 115200);
    declare_parameter<double>("angle", 180.0);  // 90 baseline->vehicle + 90 frame correction
    declare_parameter<std::string>("utm_zone", "18S");

    std::string nmea_dev = get_parameter("nmea_dev").as_string();
    int nmea_baud        = get_parameter("nmea_baud").as_int();
    std::string rtcm_dev = get_parameter("rtcm_dev").as_string();
    int rtcm_baud        = get_parameter("rtcm_baud").as_int();
    angle_               = get_parameter("angle").as_double();
    std::string utm      = get_parameter("utm_zone").as_string();
    std::strncpy(utm_zone_, utm.c_str(), sizeof(utm_zone_));
    utm_zone_[sizeof(utm_zone_) - 1] = '\0';

    LOG(INFO) << "[SEPT] NMEA on " << nmea_dev << " @ " << nmea_baud;
    LOG(INFO) << "[SEPT] RTCM on " << (rtcm_dev.empty() ? "<disabled>" : rtcm_dev) << " @ " << rtcm_baud;

    sept_ = std::make_unique<SeptentrioGPS>(nmea_dev, nmea_baud, rtcm_dev, rtcm_baud);

    // Invariant Geodetic Publishers (WGS 84 LLA maps identically)
    ant1_pub_     = create_publisher<sensor_msgs::msg::NavSatFix>("/sept/antenna1/fix", 10);

    // ENU Frame Publishers
    enu_heading_pub_     = create_publisher<std_msgs::msg::Float64>("/sept/enu/heading", 10);
    enu_heading_deg_pub_ = create_publisher<std_msgs::msg::Float64>("/sept/enu/heading_deg", 10);
    enu_orient_pub_      = create_publisher<geometry_msgs::msg::QuaternionStamped>("/sept/enu/orientation", 10);
    enu_dfix_pub_        = create_publisher<dgps_msgs::msg::DifferentialNavSatFix>("/sept/enu/dfix", 10);

    // NED Frame Publishers
    ned_heading_pub_     = create_publisher<std_msgs::msg::Float64>("/sept/ned/heading", 10);
    ned_heading_deg_pub_ = create_publisher<std_msgs::msg::Float64>("/sept/ned/heading_deg", 10);
    ned_orient_pub_      = create_publisher<geometry_msgs::msg::QuaternionStamped>("/sept/ned/orientation", 10);
    ned_dfix_pub_        = create_publisher<dgps_msgs::msg::DifferentialNavSatFix>("/sept/ned/dfix", 10);

    rtcm_sub_ = create_subscription<rtcm_msgs::msg::Message>(
        "/rtcm", 10,
        std::bind(&SeptentrioNode::rtcmCallback, this, std::placeholders::_1));

    sept_->setGpsCallback     ([this](GlobalCoord gc)   { publishGPS(gc); });
    sept_->setAttitudeCallback([this](Orientation a)    { publishHeading(a); });
    sept_->setDiffGpsCallback ([this](DiffNavSatFix d)  { publishDiffGPS(d); });

    sept_->start();
}

double SeptentrioNode::getHeadingNED(double raw_yaw_ned)
{
    // Apply vehicle offset within the NED frame (Clockwise rotation)
    double ned_heading = raw_yaw_ned + angle_ * M_PI / 180.0;
    double normalized = std::fmod(ned_heading, 2.0 * M_PI);
    if (normalized < 0) normalized += 2.0 * M_PI;
    return normalized;
}

double SeptentrioNode::getHeadingENU(double raw_yaw_ned)
{
    // Transform aligned NED heading into ENU space (0=East, Counter-Clockwise)
    double ned_heading = getHeadingNED(raw_yaw_ned);
    double enu_heading = M_PI / 2.0 - ned_heading;
    double normalized = std::fmod(enu_heading, 2.0 * M_PI);
    if (normalized < 0) normalized += 2.0 * M_PI;
    return normalized;
}

void SeptentrioNode::rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg)
{
    if (!sept_ || msg->message.empty()) return;
    sept_->write(msg->message);
}

void SeptentrioNode::publishGPS(GlobalCoord gc)
{
    sensor_msgs::msg::NavSatFix m;
    m.header.stamp = now();
    m.header.frame_id = "gps";

    m.latitude  = gc.latitude;
    m.longitude = gc.longitude;
    m.altitude  = gc.altitude;

    m.position_covariance[0] = gc.covariance.x;
    m.position_covariance[4] = gc.covariance.y;
    m.position_covariance[8] = gc.covariance.z;
    
    m.position_covariance_type = (gc.covariance.x > 0.0)
        ? sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN
        : sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    m.status.status = static_cast<int8_t>(gc.status);
    ant1_pub_->publish(m);
}

void SeptentrioNode::publishHeading(Orientation att)
{
    if (!std::isfinite(att.pry.z))
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();

        std_msgs::msg::Float64 msg;
        msg.data = nan;

        enu_heading_pub_->publish(msg);
        enu_heading_deg_pub_->publish(msg);
        ned_heading_pub_->publish(msg);
        ned_heading_deg_pub_->publish(msg);

        return;
    }
    double heading_ned = getHeadingNED(att.pry.z);
    double heading_enu = getHeadingENU(att.pry.z);

    // 1. Publish ENU Float64 Headings
    std_msgs::msg::Float64 enu_h, enu_h_deg;
    enu_h.data = heading_enu;
    enu_h_deg.data = heading_enu * 180.0 / M_PI;
    enu_heading_pub_->publish(enu_h);
    enu_heading_deg_pub_->publish(enu_h_deg);

    // 2. Publish NED Float64 Headings
    std_msgs::msg::Float64 ned_h, ned_h_deg;
    ned_h.data = heading_ned;
    ned_h_deg.data = heading_ned * 180.0 / M_PI;
    ned_heading_pub_->publish(ned_h);
    ned_heading_deg_pub_->publish(ned_h_deg);

    // 3. Publish ENU Quaternion Stamped
    tf2::Quaternion q_enu;
    // ENU Convention: Pitch flipped, using ENU heading
    q_enu.setRPY(att.pry.y, -att.pry.x, heading_enu);
    q_enu.normalize();

    geometry_msgs::msg::QuaternionStamped enu_qmsg;
    enu_qmsg.header.stamp = now();
    enu_qmsg.header.frame_id = "base_link";
    enu_qmsg.quaternion.x = q_enu.x();
    enu_qmsg.quaternion.y = q_enu.y();
    enu_qmsg.quaternion.z = q_enu.z();
    enu_qmsg.quaternion.w = q_enu.w();
    enu_orient_pub_->publish(enu_qmsg);

    // 4. Publish NED Quaternion Stamped
    tf2::Quaternion q_ned;
    q_ned.setRPY(att.pry.y, att.pry.x, att.pry.z);
    tf2::Quaternion rot_ned;
    rot_ned.setRPY(0, 0, angle_ * M_PI / 180.0);
    tf2::Quaternion result_ned = rot_ned * q_ned;
    result_ned.normalize();

    geometry_msgs::msg::QuaternionStamped ned_qmsg;
    ned_qmsg.header.stamp = now();
    ned_qmsg.header.frame_id = "ned";
    ned_qmsg.quaternion.x = result_ned.x();
    ned_qmsg.quaternion.y = result_ned.y();
    ned_qmsg.quaternion.z = result_ned.z();
    ned_qmsg.quaternion.w = result_ned.w();
    ned_orient_pub_->publish(ned_qmsg);
}

void SeptentrioNode::publishDiffGPS(DiffNavSatFix d)
{
    LOG_FIRST_N(INFO, 1) << "[SEPT] Publishing combined fix + heading";

    GlobalCoord nmea = d.gps;
    Orientation att  = d.orientation;

    double heading_enu = std::numeric_limits<double>::quiet_NaN();
    double heading_ned = std::numeric_limits<double>::quiet_NaN();

    if (std::isfinite(att.pry.z))
    {
        heading_enu = getHeadingENU(att.pry.z);
        heading_ned = getHeadingNED(att.pry.z);
    }

    const bool cov_known = nmea.covariance.x > 0.0;
    const auto cov_type = cov_known
        ? sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN
        : sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    // 1. Build Base Message Variant Container
    dgps_msgs::msg::DifferentialNavSatFix dmsg_base;
    dmsg_base.nmea.latitude  = nmea.latitude;
    dmsg_base.nmea.longitude = nmea.longitude;
    dmsg_base.nmea.altitude  = nmea.altitude;
    dmsg_base.nmea.position_covariance.fill(0.0);
    dmsg_base.nmea.position_covariance[0] = nmea.covariance.x;
    dmsg_base.nmea.position_covariance[4] = nmea.covariance.y;
    dmsg_base.nmea.position_covariance[8] = nmea.covariance.z;
    dmsg_base.nmea.status.status = nmea.status;
    dmsg_base.nmea.position_covariance_type = cov_type;
    dmsg_base.heading_covariance = static_cast<float>(att.cov.z);

    // 2. Publish to ENU Custom Fix Topic
    dgps_msgs::msg::DifferentialNavSatFix dmsg_enu = dmsg_base;
    dmsg_enu.heading     = static_cast<float>(heading_enu);
    dmsg_enu.heading_deg = static_cast<float>(heading_enu * 180.0 / M_PI);
    enu_dfix_pub_->publish(dmsg_enu);

    // 3. Publish to NED Custom Fix Topic
    dgps_msgs::msg::DifferentialNavSatFix dmsg_ned = dmsg_base;
    dmsg_ned.heading     = static_cast<float>(heading_ned);
    dmsg_ned.heading_deg = static_cast<float>(heading_ned * 180.0 / M_PI);
    ned_dfix_pub_->publish(dmsg_ned);
}

RCLCPP_COMPONENTS_REGISTER_NODE(dgps::SeptentrioNode)
