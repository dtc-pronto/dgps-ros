

#include "ros/dgps_node.hpp"


using namespace dgps;

DGPSNode::DGPSNode(const rclcpp::NodeOptions& options) : Node("dgps_node", options) 
{
    declare_parameter<std::string>("dev", "/dev/ttyACM0");
    declare_parameter<int>("baud", 460800);
    declare_parameter<double>("baseline", 0.5);
    declare_parameter<double>("angle", 90.0);
    declare_parameter<std::string>("utm_zone", "18S");

    std::string dev = get_parameter("dev").as_string();
    int baud = get_parameter("baud").as_int();
    baseline_ = get_parameter("baseline").as_double();
    angle_ = get_parameter("angle").as_double();
    std::string utm_zone_str= get_parameter("utm_zone").as_string();
    std::strncpy(utm_zone_, utm_zone_str.c_str(), sizeof(utm_zone_));
    utm_zone_[sizeof(utm_zone_) - 1] = '\0';
    
    dgps_ = std::make_unique<DifferentialGPS>(dev, baud);

    LOG(INFO) << "[DGPS] Using UTM Zone: " << utm_zone_str;
    LOG(INFO) << "[DGPS] Using Angle: " << angle_;
    LOG(INFO) << "[DGPS] Using Baseline: " << baseline_;

    // GPS Fix Publishers
    LOG(INFO) << "[DGPS] Publishing Antenna 1 position on: /dgps/antenna1/fix";
    right_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/dgps/antenna1/fix", 10); // antenna 1
    LOG(INFO) << "[DGPS] Publishing Antenna 2 positon on: /dgps/antenna2/fix";
    left_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/dgps/antenna2/fix", 10); // antenna 2.0
    LOG(INFO) << "[DGPS] Publishing Antenna Average on: /dgps/center/fix";
    avg_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/dgps/center/fix", 10); // antenna average

    // ENU Frame Publishers
    LOG(INFO) << "[DGPS] Publishing ENU Heading on: /dgps/enu/heading";
    enu_heading_pub_ = create_publisher<std_msgs::msg::Float64>("/dgps/enu/heading", 10);
    LOG(INFO) << "[DGPS] Publishing ENU Attitude on: /dgps/enu/orientation";
    enu_orient_pub_  = create_publisher<geometry_msgs::msg::QuaternionStamped>("/dgps/enu/orientation", 10);
    LOG(INFO) << "[DGPS] Publishing ENU Combined Fix on: /dgps/enu/dfix";
    enu_dgps_pub_    = create_publisher<dgps_msgs::msg::DifferentialNavSatFix>("/dgps/enu/dfix", 10);

    // NED Frame Publishers
    LOG(INFO) << "[DGPS] Publishing NED Heading on: /dgps/ned/heading";
    ned_heading_pub_ = create_publisher<std_msgs::msg::Float64>("/dgps/ned/heading", 10);
    LOG(INFO) << "[DGPS] Publishing NED Attitude on: /dgps/ned/orientation";
    ned_orient_pub_  = create_publisher<geometry_msgs::msg::QuaternionStamped>("/dgps/ned/orientation", 10);
    LOG(INFO) << "[DGPS] Publishing NED Combined Fix on: /dgps/ned/dfix";
    ned_dgps_pub_    = create_publisher<dgps_msgs::msg::DifferentialNavSatFix>("/dgps/ned/dfix", 10);
    
    rtcm_sub_ = create_subscription<rtcm_msgs::msg::Message>("/rtcm", 10, std::bind(&DGPSNode::rtcmCallback, this, std::placeholders::_1));

    dgps_->setGpsCallback([this](dgps::GlobalCoord nmea) { this->publishGPS(nmea); });
    dgps_->setAttitudeCallback([this](dgps::Orientation attitude) { this->publishHeading(attitude); });
    dgps_->setDiffGpsCallback([this](dgps::DiffNavSatFix dnsf) { this->publishDiffGPS(dnsf); });

    dgps_->start();
}

void DGPSNode::rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg)
{
    if (!dgps_ || msg->message.empty()) return;
    dgps_->write(msg->message);
}

void DGPSNode::publishGPS(dgps::GlobalCoord nmea)
{
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = now();
    msg.header.frame_id = "gps";

    msg.latitude = nmea.latitude;
    msg.longitude = nmea.longitude;
    msg.altitude = nmea.altitude;

    msg.position_covariance[0] = nmea.covariance.x;
    msg.position_covariance[4] = nmea.covariance.y;
    msg.position_covariance[8] = nmea.covariance.z;

    msg.status.status = static_cast<int8_t>(nmea.status);
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    right_fix_pub_->publish(msg);
    double temp1, temp2;
    geodetics::LLtoUTM(nmea.latitude, nmea.longitude, temp1, temp2, utm_zone_);
}

double DGPSNode::getHeadingNED(double raw_yaw_ned)
{
    // Apply vehicle alignment offset in NED space (Clockwise rotation)
    double ned_heading = raw_yaw_ned + angle_ * M_PI / 180.0;
    double normalized = std::fmod(ned_heading, 2.0 * M_PI);
    if (normalized < 0) normalized += 2.0 * M_PI;
    return normalized;
}

double DGPSNode::getHeadingENU(double raw_yaw_ned)
{
    // Get aligned NED heading, then convert to ENU (0=East, Counter-Clockwise)
    double ned_heading = getHeadingNED(raw_yaw_ned);
    double enu_heading = (M_PI / 2.0) - ned_heading;
    double normalized = std::fmod(enu_heading, 2.0 * M_PI);
    if (normalized < 0) normalized += 2.0 * M_PI;
    return normalized;
}

void DGPSNode::publishHeading(dgps::Orientation attitude)
{
    double heading_ned = getHeadingNED(attitude.pry.z);
    double heading_enu = getHeadingENU(attitude.pry.z);

    // Publish Headings
    std_msgs::msg::Float64 enu_hmsg, ned_hmsg;
    enu_hmsg.data = heading_enu;
    ned_hmsg.data = heading_ned;
    enu_heading_pub_->publish(enu_hmsg);
    ned_heading_pub_->publish(ned_hmsg);
    
    // Publish ENU Quaternion Stamped
    tf2::Quaternion q_enu;
    // ROS ENU convention: Roll (X-forward), Pitch (Y-left), Yaw (Z-up)
    q_enu.setRPY(attitude.pry.y, attitude.pry.x, heading_enu);
    q_enu.normalize();

    geometry_msgs::msg::QuaternionStamped enu_qmsg;
    enu_qmsg.header.stamp = now();
    enu_qmsg.header.frame_id = "base_link"; // Standard ROS ENU vehicle frame
    enu_qmsg.quaternion.x = q_enu.x();
    enu_qmsg.quaternion.y = q_enu.y();
    enu_qmsg.quaternion.z = q_enu.z();
    enu_qmsg.quaternion.w = q_enu.w();
    enu_orient_pub_->publish(enu_qmsg);

    // Publish NED Quaternion Stamped
    tf2::Quaternion q_ned;
    // Roll (X-forward), Pitch (Y-right), Yaw (Z-down)
    q_ned.setRPY(attitude.pry.y, attitude.pry.x, heading_ned);
    q_ned.normalize();

    geometry_msgs::msg::QuaternionStamped ned_qmsg;
    ned_qmsg.header.stamp = now();
    ned_qmsg.header.frame_id = "ned";
    ned_qmsg.quaternion.x = q_ned.x();
    ned_qmsg.quaternion.y = q_ned.y();
    ned_qmsg.quaternion.z = q_ned.z();
    ned_qmsg.quaternion.w = q_ned.w();
    ned_orient_pub_->publish(ned_qmsg);
}

void DGPSNode::publishDiffGPS(dgps::DiffNavSatFix dgps)
{
    LOG_FIRST_N(INFO, 1) << "[DGPS] Starting to publish fix and heading combined";
    dgps::GlobalCoord nmea = dgps.gps;
    dgps::Orientation attitude = dgps.orientation;

    // Use ENU heading for localized physical UTM projection math 
    double heading_enu = getHeadingENU(attitude.pry.z);
    double heading_ned = getHeadingNED(attitude.pry.z);

    double utm_northing, utm_easting;
    geodetics::LLtoUTM(nmea.latitude, nmea.longitude, utm_northing, utm_easting, utm_zone_);

    // Baseline calculation stays mapped to UTM grid projection coordinates
    double dx = -baseline_ * sin(heading_enu);
    double dy =  baseline_ * cos(heading_enu);
    utm_easting  += dx;
    utm_northing += dy;

    double left_lat, left_lon;
    geodetics::UTMtoLL(utm_northing, utm_easting, utm_zone_, left_lat, left_lon);
    double left_alt = nmea.altitude;

    sensor_msgs::msg::NavSatFix left_msg;
    left_msg.header.stamp = now();
    left_msg.header.frame_id = "gps";

    left_msg.latitude  = left_lat;
    left_msg.longitude = left_lon;
    left_msg.altitude  = left_alt;

    left_msg.position_covariance.fill(0.0);
    left_msg.position_covariance[0] = nmea.covariance.x;
    left_msg.position_covariance[4] = nmea.covariance.y;
    left_msg.position_covariance[8] = nmea.covariance.z;

    left_msg.status.status = nmea.status;

    left_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    left_fix_pub_->publish(left_msg);

    sensor_msgs::msg::NavSatFix avg_msg;
    avg_msg.header = left_msg.header;

    avg_msg.latitude  = (left_lat  + nmea.latitude)  * 0.5;
    avg_msg.longitude = (left_lon  + nmea.longitude) * 0.5;
    avg_msg.altitude  = (left_alt  + nmea.altitude)  * 0.5;

    avg_msg.position_covariance = left_msg.position_covariance;
    avg_msg.status = left_msg.status;
    avg_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    avg_fix_pub_->publish(avg_msg);

    // Publish ENU Custom Message
    dgps_msgs::msg::DifferentialNavSatFix enu_dgps_msg;
    enu_dgps_msg.nmea.latitude = nmea.latitude;
    enu_dgps_msg.nmea.longitude = nmea.longitude;
    enu_dgps_msg.nmea.altitude = nmea.altitude;
    enu_dgps_msg.nmea.status.status = nmea.status;
    enu_dgps_msg.nmea.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    enu_dgps_msg.nmea.position_covariance[0] = nmea.covariance.x;
    enu_dgps_msg.nmea.position_covariance[4] = nmea.covariance.y;
    enu_dgps_msg.nmea.position_covariance[8] = nmea.covariance.z;

    enu_dgps_msg.heading = static_cast<float>(heading_enu);
    enu_dgps_msg.heading_deg = static_cast<float>(heading_enu * 180.0 / M_PI);
    enu_dgps_msg.heading_covariance = static_cast<float>(attitude.cov.z);
    enu_dgps_pub_->publish(enu_dgps_msg);

    // Publish NED Custom Message
    dgps_msgs::msg::DifferentialNavSatFix ned_dgps_msg = enu_dgps_msg; // Copy invariant data
    ned_dgps_msg.heading = static_cast<float>(heading_ned);
    ned_dgps_msg.heading_deg = static_cast<float>(heading_ned * 180.0 / M_PI);
    ned_dgps_pub_->publish(ned_dgps_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(dgps::DGPSNode)

