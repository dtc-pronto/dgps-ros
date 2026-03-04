

#include "ros/dgps_node.hpp"


using namespace dgps;

DGPSNode::DGPSNode() : Node("dgps_node") 
{
    declare_parameter<std::string>("dev", "/dev/ttyACM0");
    declare_parameter<int>("baud", 460800);
    declare_parameter<double>("baseline", 0.5);

    std::string dev = get_parameter("dev").as_string();
    int baud = get_parameter("baud").as_int();
    baseline_ = get_parameter("baseline").as_double();

    right_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/dgps/right/fix", 10); // antenna 1
    left_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/dgps/left/fix", 10); // antenna 2
    avg_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/dgps/center/fix", 10); // antenna average

    heading_pub_ = create_publisher<std_msgs::msg::Float64>("/dgps/heading", 10);
    orient_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>("/dgps/orientation", 10);
    
    rtcm_sub_ = create_subscription<rtcm_msgs::msg::Message>("/rtcm", 10, std::bind(&DGPSNode::rtcmCallback, this, std::placeholders::_1));

    dgps_ = std::make_unique<DifferentialGPS>(dev, baud);

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
    msg.header.frame_id = "enu";

    msg.latitude = nmea.latitude;
    msg.longitude = nmea.longitude;
    msg.altitude = nmea.altitude;

    msg.position_covariance[0] = nmea.covariance.x;
    msg.position_covariance[4] = nmea.covariance.y;
    msg.position_covariance[8] = nmea.covariance.z;

    msg.status.status = (nmea.status > 0) ?
            sensor_msgs::msg::NavSatStatus::STATUS_FIX :
            sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    msg.position_covariance_type=sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    right_fix_pub_->publish(msg);
}

void DGPSNode::publishHeading(dgps::Orientation attitude)
{
    std_msgs::msg::Float64 hmsg;
    hmsg.data = attitude.pry.z;
    heading_pub_->publish(hmsg);
    
    tf2::Quaternion q;
    geometry_msgs::msg::QuaternionStamped qmsg;

    q.setRPY(attitude.pry.y, attitude.pry.x, attitude.pry.z);

    qmsg.header.stamp = now();
    qmsg.header.frame_id = "enu";

    qmsg.quaternion.x = q.x();
    qmsg.quaternion.y = q.y();
    qmsg.quaternion.z = q.z();
    qmsg.quaternion.w = q.w();

    orient_pub_->publish(qmsg);
}

void DGPSNode::publishDiffGPS(dgps::DiffNavSatFix dgps)
{
    // Separate position and heading data
    dgps::GlobalCoord nmea = dgps.gps;
    dgps::Orientation attitude = dgps.orientation;

    // Compute left antenna position
    double heading  = attitude.pry.z * M_PI / 180.0; // radians
    double baseline = dgps.orientation.baseline;                 // meters

    double dx = -baseline_ * sin(heading);  // East offset (meters)
    double dy =  baseline_ * cos(heading);  // North offset (meters)

    // Convert meter offsets to lat/lon
    constexpr double R = 6378137.0; // WGS84 Earth radius (meters)
    double dLat = dy / R; // Latitude offset (radians)

    double lat_rad = nmea.latitude * M_PI / 180.0; // Latitude in radians (used for longitude scaling)
    double dLon = dx / (R * cos(lat_rad)); // Longitude offset (radians)

    double left_lat = nmea.latitude  + dLat * 180.0 / M_PI;
    double left_lon = nmea.longitude + dLon * 180.0 / M_PI;
    double left_alt = nmea.altitude;
    
    // Create and publish left fix message
    sensor_msgs::msg::NavSatFix left_msg;
    left_msg.header.stamp = now();
    left_msg.header.frame_id = "enu";

    left_msg.latitude  = left_lat;
    left_msg.longitude = left_lon;
    left_msg.altitude  = left_alt;

    left_msg.position_covariance.fill(0.0);
    left_msg.position_covariance[0] = nmea.covariance.x;
    left_msg.position_covariance[4] = nmea.covariance.y;
    left_msg.position_covariance[8] = nmea.covariance.z;

    left_msg.status.status =
        (nmea.status > 0) ?
        sensor_msgs::msg::NavSatStatus::STATUS_FIX :
        sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

    left_msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    left_fix_pub_->publish(left_msg);

    // Create and publish average fix message
    sensor_msgs::msg::NavSatFix avg_msg;
    avg_msg.header = left_msg.header;

    avg_msg.latitude  = (left_lat  + nmea.latitude)  * 0.5;
    avg_msg.longitude = (left_lon  + nmea.longitude) * 0.5;
    avg_msg.altitude  = (left_alt  + nmea.altitude)  * 0.5;

    avg_msg.position_covariance = left_msg.position_covariance;
    avg_msg.status = left_msg.status;
    avg_msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    avg_fix_pub_->publish(avg_msg);
}
