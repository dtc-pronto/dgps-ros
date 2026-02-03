#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <serial/serial.h>

#include <string>
#include <vector>
#include <sstream>
#include <cmath>

static std::vector<std::string> split(const std::string &s, char d)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, d)) out.push_back(item);
  return out;
}

class Diff_GPS_node : public rclcpp::Node
{
public:
  Diff_GPS_node() : Node("diff_GPS_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0: USB Dual_Serial");
    baud_ = declare_parameter<int>("baudrate", 460800);

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
    heading_pub_ =
      create_publisher<geometry_msgs::msg::QuaternionStamped>("gps/heading", 10);

    try {
      serial_.setPort(port_);
      serial_.setBaudrate(baud_);
      serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
      serial_.open();
    } catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "Serial open failed: %s", e.what());
      throw;
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Diff_GPS_node::read_serial, this));
  }

private:
  void read_serial()
  {
    if (!serial_.isOpen()) return;
    std::string line = serial_.readline(4096, "\n");
    if (line.empty() || line[0] != '$') return;

    if (line.rfind("$GNGGA", 0) == 0) {
      parse_gga(line);
    } else if (line.rfind("$HDT", 0) == 0 || line.rfind("$GNVTG", 0) == 0) {
      parse_heading(line);
    }
  }

  void parse_gga(const std::string &nmea)
  {
    auto f = split(nmea, ',');
    if (f.size() < 10 || f[2].empty()) return;

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
  }

  void parse_heading(const std::string &nmea)
  {
    auto f = split(nmea, ',');
    if (f.size() < 2 || f[1].empty()) return;

    double heading_deg = std::stod(f[1]);
    double yaw = heading_deg * M_PI / 180.0;

    geometry_msgs::msg::QuaternionStamped q;
    q.header.stamp = now();
    q.header.frame_id = "gps";
    q.quaternion.z = std::sin(yaw * 0.5);
    q.quaternion.w = std::cos(yaw * 0.5);

    heading_pub_->publish(q);
  }

  static double nmea_to_deg(const std::string &val, const std::string &dir)
  {
    if (val.size() < 6) return 0.0;

    int deg_len = (dir == "N" || dir == "S") ? 2 : 3;
    double deg = std::stod(val.substr(0, deg_len));
    double min = std::stod(val.substr(deg_len));
    double out = deg + min / 60.0;

    if (dir == "S" || dir == "W") out *= -1.0;
    return out;
  }

  std::string port_;
  int baud_;
  serial::Serial serial_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Diff_GPS_node>());
  rclcpp::shutdown();
  return 0;
}
