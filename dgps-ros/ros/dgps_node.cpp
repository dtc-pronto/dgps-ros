

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

    LOG(INFO) << "[DGPS] Publishing Antenna 1 position on: /dgps/antenna1/fix";
    right_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/dgps/antenna1/fix", 10); // antenna 1
    LOG(INFO) << "[DGPS] Publishing Antenna 2 positon on: /dgps/antenna2/fix";
    left_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/dgps/antenna2/fix", 10); // antenna 2.0
    LOG(INFO) << "[DGPS] Publishing Antenna Average on: /dgps/center/fix";
    avg_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/dgps/center/fix", 10); // antenna average

    LOG(INFO) << "[DGPS] Publishing ENU Heading on: /dgps/heading";
    heading_pub_ = create_publisher<std_msgs::msg::Float64>("/dgps/heading", 10);
    LOG(INFO) << "[DGPS] Publishing ENU Heading Stamped on: /dgps/heading/stamped";
    heading_stamp_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/dgps/heading/stamped", 10);
    LOG(INFO) << "[DGPS] Publishing Attitude on: /dgps/orientation";
    orient_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>("/dgps/orientation", 10);

    LOG(INFO) << "[DGPS] Publishing Combined GPS and Heading on: /dgps/dfix";
    dgps_pub_ = create_publisher<gps_msgs::msg::GPSFix>("/dgps/dfix", 10);
    
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
    msg.header = nmeaDoubleTimeToStampTime(nmea.timestamp);

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

double DGPSNode::transformHeading(double heading)
{
    // rotate to align with vehicle
    double vehicle_heading = heading + angle_ * M_PI / 180.0;

    // ned -> enu
    double enu_heading = M_PI / 2.0 - vehicle_heading;
    double normalized_enu = std::fmod(enu_heading, 2.0*M_PI);
    if (normalized_enu < 0) normalized_enu += 2.0 * M_PI;

    return normalized_enu;
}

void DGPSNode::publishHeading(dgps::Orientation attitude)
{
    std_msgs::msg::Float64 hmsg;
    hmsg.data = transformHeading(attitude.pry.z);
    heading_pub_->publish(hmsg);
    
    geometry_msgs::msg::PointStamped hsmsg;
    hsmsg.header = nmeaDoubleTimeToStampTime(attitude.timestamp);
    hsmsg.point.z = attitude.pry.z;

    tf2::Quaternion q;
    geometry_msgs::msg::QuaternionStamped qmsg;

    tf2::Quaternion rot;
    rot.setRPY(0, 0, angle_ * M_PI/180.0);

    q.setRPY(attitude.pry.y, attitude.pry.x, attitude.pry.z);

    tf2::Quaternion result = rot * q;
    q.normalize();

    qmsg.header.stamp = now();
    qmsg.header.frame_id = "ned";

    qmsg.quaternion.x = result.x();
    qmsg.quaternion.y = result.y();
    qmsg.quaternion.z = result.z();
    qmsg.quaternion.w = result.w();

    orient_pub_->publish(qmsg);
}

void DGPSNode::publishDiffGPS(dgps::DiffNavSatFix dgps)
{
    LOG_FIRST_N(INFO, 1) << "[DGPS] Starting to publish fix and heading combined";
    dgps::GlobalCoord nmea = dgps.gps;
    dgps::Orientation attitude = dgps.orientation;

    double heading  = transformHeading(attitude.pry.z);
    double utm_northing, utm_easting;
    geodetics::LLtoUTM(nmea.latitude, nmea.longitude, utm_northing, utm_easting, utm_zone_);

    double dx = -baseline_ * sin(heading);
    double dy =  baseline_ * cos(heading);
    utm_easting  += dx;
    utm_northing += dy;

    double left_lat, left_lon;
    geodetics::UTMtoLL(utm_northing, utm_easting, utm_zone_, left_lat, left_lon);
    double left_alt = nmea.altitude;

    sensor_msgs::msg::NavSatFix left_msg;
    left_msg.header = nmeaDoubleTimeToStampTime(dgps.gps.timestamp);
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

    gps_msgs::msg::GPSFix dgps_msg;

    dgps_msg.header = left_msg.header;
    dgps_msg.latitude = nmea.latitude;
    dgps_msg.longitude = nmea.longitude;
    dgps_msg.altitude = nmea.altitude;
     
    dgps_msg.err = nmea.covariance.x;
    dgps_msg.err_horz = nmea.covariance.y;
    dgps_msg.err_vert = nmea.covariance.z;

    dgps_msg.status.status = nmea.status;

    dgps_msg.track = heading;
    dgps_msg.err_track = attitude.cov.z;

    dgps_pub_->publish(dgps_msg);
}

std_msgs::msg::Header DGPSNode::nmeaDoubleTimeToStampTime(double utc_time)
{
    int hh = static_cast<int>(utc_time / 10000);
    int mm = static_cast<int>(std::fmod(utc_time, 10000) / 100);
    double ss_frac = std::fmod(utc_time, 100);

    auto now = this->now();
    int64_t now_secs = now.seconds();
    int64_t midnight = (now_secs / 86400) * 86400;

    int32_t epoch_secs = static_cast<int32_t>(midnight)
                       + hh * 3600 + mm * 60 + static_cast<int>(ss_frac);
    uint32_t nanosec = static_cast<uint32_t>((ss_frac - std::floor(ss_frac)) * 1e9);

    std_msgs::msg::Header header;
    header.stamp.sec = epoch_secs;
    header.stamp.nanosec = nanosec;
    header.frame_id = "gps";
    return header;
}

RCLCPP_COMPONENTS_REGISTER_NODE(dgps::DGPSNode)

