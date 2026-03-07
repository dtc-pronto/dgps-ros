#include <rclcpp/rclcpp.hpp>

// Standard ROS messages for GPS
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

// RTCM message type for RTK corrections
#include <rtcm_msgs/msg/message.hpp>

// Serial communication library
#include <libserial/SerialPort.h>

#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

/* ------------------ Utility Function ------------------ */
/**
 * @brief Split a string by a given delimiter
 * @param s Input string
 * @param delim Delimiter character
 * @return Vector of split strings
 */
static std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        out.push_back(item);
    }
    return out;
}

/* ------------------ DiffGpsNode Class ------------------ */
class DiffGpsNode : public rclcpp::Node
{
public:
    DiffGpsNode() : Node("diff_gps_node")
    {
        // ------------------ Parameters ------------------
        port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
        baud_ = declare_parameter<int>("baudrate", 460800);

        RCLCPP_INFO(get_logger(), "DiffGpsNode starting with port=%s baud=%d",
                    port_.c_str(), baud_);

        // ------------------ Publishers ------------------
        fix_pub_ =
            create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
        heading_pub_ =
            create_publisher<geometry_msgs::msg::QuaternionStamped>("gps/heading", 10);

        // ------------------ Subscriber for RTCM Corrections ------------------
        rtcm_sub_ = create_subscription<rtcm_msgs::msg::Message>(
            "/rtcm",
            rclcpp::QoS(10),
            std::bind(&DiffGpsNode::rtcm_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(get_logger(), "Subscribed to /rtcm corrections");

        // ------------------ Serial Port Initialization ------------------
        try {
            open_serial();
        } catch (const std::exception &e) {
            RCLCPP_FATAL(get_logger(), "Failed to open serial port: %s", e.what());
            throw;
        }

        // ------------------ Timer for Reading Serial ------------------
        timer_ = create_wall_timer(
            100ms, std::bind(&DiffGpsNode::read_serial, this));
    }

private:
    /* ------------------ Serial Port Setup ------------------ */
    void open_serial()
    {
        RCLCPP_INFO(get_logger(), "Attempting to open serial port %s", port_.c_str());

        serial_.Open(port_);
        serial_.SetBaudRate(LibSerial::BaudRate::BAUD_460800);
        serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

        RCLCPP_INFO(get_logger(), "Serial port opened successfully");
    }

    /* ------------------ Read NMEA Lines from GPS ------------------ */
    void read_serial()
    {
        if (!serial_.IsOpen()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                 "Serial port not open yet");
            return;
        }

        std::string line;
        try {
            serial_.ReadLine(line, '\n', 1000); // Timeout = 1 sec
            RCLCPP_DEBUG(get_logger(), "Read line: %s", line.c_str());
        } catch (const LibSerial::ReadTimeout &) {
            RCLCPP_DEBUG(get_logger(), "Read timeout, no data available");
            return;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Exception reading serial: %s", e.what());
            return;
        }

        if (line.empty()) return;

        // Only parse NMEA sentences starting with $
        if (line[0] != '$') {
            RCLCPP_DEBUG(get_logger(), "Ignoring line: %s", line.c_str());
            return;
        }

        // Route different NMEA types
        if (line.rfind("$GNGGA", 0) == 0) {
            RCLCPP_DEBUG(get_logger(), "Parsing GGA: %s", line.c_str());
            parse_gga(line);
        } else if (line.rfind("$HDT", 0) == 0 ||
                   line.rfind("$GNVTG", 0) == 0) {
            RCLCPP_DEBUG(get_logger(), "Parsing heading: %s", line.c_str());
            parse_heading(line);
        } else {
            RCLCPP_DEBUG(get_logger(), "Unknown NMEA type: %s", line.c_str());
        }
    }

    /* ------------------ Parse GGA NMEA Sentence ------------------ */
    void parse_gga(const std::string &nmea)
    {
        auto f = split(nmea, ',');
        if (f.size() < 10 || f[2].empty()) {
            RCLCPP_WARN(get_logger(), "Invalid GGA line: %s", nmea.c_str());
            return;
        }

        double lat = nmea_to_deg(f[2], f[3]);
        double lon = nmea_to_deg(f[4], f[5]);
        double alt = std::stod(f[9]);

        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = now();
        msg.header.frame_id = "gps";

        msg.latitude = lat;
        msg.longitude = lon;
        msg.altitude = alt;

        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

        fix_pub_->publish(msg);
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                              "Published GGA: lat=%.6f lon=%.6f alt=%.2f", lat, lon, alt);
    }

    /* ------------------ Parse Heading NMEA Sentence ------------------ */
    void parse_heading(const std::string &nmea)
    {
        auto f = split(nmea, ',');
        if (f.size() < 2 || f[1].empty()) {
            RCLCPP_WARN(get_logger(), "Invalid heading line: %s", nmea.c_str());
            return;
        }

        double heading_deg = std::stod(f[1]);
        double yaw = heading_deg * M_PI / 180.0;

        geometry_msgs::msg::QuaternionStamped q;
        q.header.stamp = now();
        q.header.frame_id = "gps";

        q.quaternion.x = 0.0;
        q.quaternion.y = 0.0;
        q.quaternion.z = std::sin(yaw * 0.5);
        q.quaternion.w = std::cos(yaw * 0.5);

        heading_pub_->publish(q);
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                              "Published heading: %.2f deg", heading_deg);
    }

    /* ------------------ Convert NMEA Lat/Lon to Decimal Degrees ------------------ */
    static double nmea_to_deg(const std::string &val, const std::string &dir)
    {
        if (val.size() < 6) return 0.0;

        int deg_len = (dir == "N" || dir == "S") ? 2 : 3;
        double deg = std::stod(val.substr(0, deg_len));
        double min = std::stod(val.substr(deg_len));

        double out = deg + (min / 60.0);
        if (dir == "S" || dir == "W") out *= -1.0;
        return out;
    }

    /* ------------------ RTCM Subscriber Callback ------------------ */
    void rtcm_callback(const rtcm_msgs::msg::Message::SharedPtr msg)
    {
        if (!serial_.IsOpen()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Serial port not open, cannot forward RTCM");
            return;
        }

        if (msg->message.empty()) {
            return;
        }

        try {
            // Forward the raw RTCM bytes to the GPS
            LibSerial::DataBuffer buffer(msg->message.begin(), msg->message.end());
            serial_.Write(buffer);

            RCLCPP_DEBUG(get_logger(),
                         "Forwarded RTCM (%zu bytes) to GPS",
                         msg->message.size());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "RTCM write failed: %s", e.what());
        }
    }

    /* ------------------ Member Variables ------------------ */
    std::string port_;
    int baud_;
    LibSerial::SerialPort serial_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_pub_;
    rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;
};

/* ------------------ Main Function ------------------ */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DiffGpsNode>();
    RCLCPP_INFO(node->get_logger(), "DiffGpsNode spinning...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
