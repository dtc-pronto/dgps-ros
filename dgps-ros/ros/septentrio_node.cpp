/*!
* @Date 2026
*
* @About ROS2 component implementation for the Septentrio mosaic-G5 P3H.
* Mirrors dgps_node topic layout, prefixed with /sept.
*/
#include <cmath>
#include <cstring>

#include "ros/septentrio_node.hpp"

using namespace dgps;

SeptentrioNode::SeptentrioNode(const rclcpp::NodeOptions& options) : Node("septentrio_node", options)
{
    declare_parameter<std::string>("nmea_dev",  "/dev/ttyACM0");  // USB1
    declare_parameter<int>("nmea_baud", 115200);
    declare_parameter<std::string>("rtcm_dev",  "/dev/ttyACM1");  // USB2; set empty to disable RTCM
    declare_parameter<int>("rtcm_baud", 115200);
    declare_parameter<double>("baseline", 0.5);
    declare_parameter<double>("angle", 180.0);  // 90 baseline->vehicle + 90 frame correction
    declare_parameter<std::string>("utm_zone", "18S");

    std::string nmea_dev = get_parameter("nmea_dev").as_string();
    int nmea_baud        = get_parameter("nmea_baud").as_int();
    std::string rtcm_dev = get_parameter("rtcm_dev").as_string();
    int rtcm_baud        = get_parameter("rtcm_baud").as_int();
    baseline_param_      = get_parameter("baseline").as_double();
    angle_               = get_parameter("angle").as_double();
    std::string utm      = get_parameter("utm_zone").as_string();
    std::strncpy(utm_zone_, utm.c_str(), sizeof(utm_zone_));
    utm_zone_[sizeof(utm_zone_) - 1] = '\0';

    LOG(INFO) << "[SEPT] NMEA on " << nmea_dev << " @ " << nmea_baud;
    LOG(INFO) << "[SEPT] RTCM on " << (rtcm_dev.empty() ? "<disabled>" : rtcm_dev) << " @ " << rtcm_baud;
    LOG(INFO) << "[SEPT] UTM zone " << utm << "  baseline=" << baseline_param_ << "m  angle=" << angle_ << "deg";

    sept_ = std::make_unique<SeptentrioGPS>(nmea_dev, nmea_baud, rtcm_dev, rtcm_baud);

    ant1_pub_     = create_publisher<sensor_msgs::msg::NavSatFix>("/sept/antenna1/fix", 10);
    ant2_pub_     = create_publisher<sensor_msgs::msg::NavSatFix>("/sept/antenna2/fix", 10);
    center_pub_   = create_publisher<sensor_msgs::msg::NavSatFix>("/sept/center/fix",   10);
    dfix_pub_     = create_publisher<dgps_msgs::msg::DifferentialNavSatFix>("/sept/dfix", 10);
    heading_pub_     = create_publisher<std_msgs::msg::Float64>("/sept/heading", 10);
    heading_deg_pub_ = create_publisher<std_msgs::msg::Float64>("/sept/heading_deg", 10);
    orient_pub_   = create_publisher<geometry_msgs::msg::QuaternionStamped>("/sept/orientation", 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/sept/baseline_velocity", 10);

    rtcm_sub_ = create_subscription<rtcm_msgs::msg::Message>(
        "/rtcm", 10,
        std::bind(&SeptentrioNode::rtcmCallback, this, std::placeholders::_1));

    sept_->setGpsCallback     ([this](GlobalCoord gc)   { publishGPS(gc); });
    sept_->setAttitudeCallback([this](Orientation a)    { publishHeading(a); });
    sept_->setBaselineCallback([this](Baseline b)       { publishBaseline(b); });
    sept_->setVelocityCallback([this](Velocity v)       { publishVelocity(v); });
    sept_->setDiffGpsCallback ([this](DiffNavSatFix d)  { publishDiffGPS(d); });

    sept_->start();
}

double SeptentrioNode::transformHeading(double heading)
{
    double vehicle_heading = heading + angle_ * M_PI / 180.0;
    double enu = M_PI / 2.0 - vehicle_heading;
    double normalized = std::fmod(enu, 2.0 * M_PI);
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
    // GST populates the covariance; before the first GST it is left at zero.
    m.position_covariance_type = (gc.covariance.x > 0.0)
        ? sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN
        : sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    m.status.status = static_cast<int8_t>(gc.status);
    ant1_pub_->publish(m);
}

void SeptentrioNode::publishHeading(Orientation att)
{
    double heading_rad = transformHeading(att.pry.z);
    std_msgs::msg::Float64 h;
    h.data = heading_rad;
    heading_pub_->publish(h);

    std_msgs::msg::Float64 h_deg;
    h_deg.data = heading_rad * 180.0 / M_PI;
    heading_deg_pub_->publish(h_deg);

    tf2::Quaternion q;
    q.setRPY(att.pry.y, att.pry.x, att.pry.z);  // (roll, pitch, yaw) — matches dgps_node ordering
    tf2::Quaternion rot;
    rot.setRPY(0, 0, angle_ * M_PI / 180.0);
    tf2::Quaternion result = rot * q;

    geometry_msgs::msg::QuaternionStamped qmsg;
    qmsg.header.stamp = now();
    qmsg.header.frame_id = "ned";
    qmsg.quaternion.x = result.x();
    qmsg.quaternion.y = result.y();
    qmsg.quaternion.z = result.z();
    qmsg.quaternion.w = result.w();
    orient_pub_->publish(qmsg);
}

void SeptentrioNode::publishBaseline(Baseline b)
{
    last_baseline_ = std::make_unique<Baseline>(b);
}

void SeptentrioNode::publishVelocity(Velocity v)
{
    // RBV is the rate of change of the rover->base baseline vector, NOT
    // vehicle ground velocity. On a rigid dual-antenna mount this is driven
    // by vehicle rotation. Published in local ENU.
    geometry_msgs::msg::TwistStamped t;
    t.header.stamp = now();
    t.header.frame_id = "enu";
    t.twist.linear.x = v.v.x;  // east
    t.twist.linear.y = v.v.y;  // north
    t.twist.linear.z = v.v.z;  // up
    velocity_pub_->publish(t);
}

void SeptentrioNode::publishDiffGPS(DiffNavSatFix d)
{
    LOG_FIRST_N(INFO, 1) << "[SEPT] Publishing combined fix + heading";

    GlobalCoord nmea = d.gps;
    Orientation att  = d.orientation;

    double heading = transformHeading(att.pry.z);

    double utm_n, utm_e;
    geodetics::LLtoUTM(nmea.latitude, nmea.longitude, utm_n, utm_e, utm_zone_);

    // Prefer the measured baseline vector from RBP; fall back to baseline+heading param.
    double dE, dN;
    if (last_baseline_ && (last_baseline_->delta.x != 0.0 || last_baseline_->delta.y != 0.0))
    {
        dE = last_baseline_->delta.x;
        dN = last_baseline_->delta.y;
    }
    else
    {
        dE = -baseline_param_ * std::sin(heading);
        dN =  baseline_param_ * std::cos(heading);
    }
    double ant2_e = utm_e + dE;
    double ant2_n = utm_n + dN;

    double ant2_lat, ant2_lon;
    geodetics::UTMtoLL(ant2_n, ant2_e, utm_zone_, ant2_lat, ant2_lon);

    const bool cov_known = nmea.covariance.x > 0.0;
    const auto cov_type = cov_known
        ? sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN
        : sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    sensor_msgs::msg::NavSatFix ant2;
    ant2.header.stamp = now();
    ant2.header.frame_id = "gps";
    ant2.latitude  = ant2_lat;
    ant2.longitude = ant2_lon;
    ant2.altitude  = nmea.altitude;
    ant2.position_covariance.fill(0.0);
    ant2.position_covariance[0] = nmea.covariance.x;
    ant2.position_covariance[4] = nmea.covariance.y;
    ant2.position_covariance[8] = nmea.covariance.z;
    ant2.status.status = nmea.status;
    ant2.position_covariance_type = cov_type;
    ant2_pub_->publish(ant2);

    sensor_msgs::msg::NavSatFix center;
    center.header = ant2.header;
    center.latitude  = (ant2_lat + nmea.latitude)  * 0.5;
    center.longitude = (ant2_lon + nmea.longitude) * 0.5;
    center.altitude  = nmea.altitude;
    center.position_covariance = ant2.position_covariance;
    center.status = ant2.status;
    center.position_covariance_type = cov_type;
    center_pub_->publish(center);

    dgps_msgs::msg::DifferentialNavSatFix dmsg;
    dmsg.nmea.latitude  = nmea.latitude;
    dmsg.nmea.longitude = nmea.longitude;
    dmsg.nmea.altitude  = nmea.altitude;
    dmsg.nmea.position_covariance.fill(0.0);
    dmsg.nmea.position_covariance[0] = nmea.covariance.x;
    dmsg.nmea.position_covariance[4] = nmea.covariance.y;
    dmsg.nmea.position_covariance[8] = nmea.covariance.z;
    dmsg.nmea.status.status = nmea.status;
    dmsg.nmea.position_covariance_type = cov_type;
    dmsg.heading = static_cast<float>(heading);
    dmsg.heading_deg = static_cast<float>(heading * 180.0 / M_PI);
    dmsg.heading_covariance = static_cast<float>(att.cov.z);
    dfix_pub_->publish(dmsg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(dgps::SeptentrioNode)
