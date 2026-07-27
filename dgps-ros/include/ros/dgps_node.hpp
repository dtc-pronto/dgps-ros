/*!
* @Authors Ethan Sanchez, Jason Hughes
* @Date March 2026
*
* @About Handle all the ros stuff
*/
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rtcm_msgs/msg/message.hpp>
#include <dgps_msgs/msg/differential_nav_sat_fix.hpp>

#include "dgps/differential_gps.hpp"
#include "dgps/geodetics.hpp"


namespace dgps
{
class DGPSNode : public rclcpp::Node 
{
    public:
        DGPSNode(const rclcpp::NodeOptions& options);

    private: 
              
        double baseline_;
        double angle_;
        char utm_zone_[10];

        // Coordinate-specific heading transformation helpers
        double getHeadingNED(double raw_yaw_ned);
        double getHeadingENU(double raw_yaw_ned);

        void publishGPS(dgps::GlobalCoord nmea);
        void publishHeading(dgps::Orientation attitude);
        void publishDiffGPS(dgps::DiffNavSatFix dgps);

        void rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg);

        std::unique_ptr<DifferentialGPS> dgps_;

        // Baseline global position fix publishers
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr right_fix_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr left_fix_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr avg_fix_pub_;

        // ENU Frame publishers
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr enu_heading_pub_;
        rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr enu_orient_pub_;
        rclcpp::Publisher<dgps_msgs::msg::DifferentialNavSatFix>::SharedPtr enu_dgps_pub_;  

        // NED Frame publishers
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ned_heading_pub_;
        rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr ned_orient_pub_;
        rclcpp::Publisher<dgps_msgs::msg::DifferentialNavSatFix>::SharedPtr ned_dgps_pub_;  

        rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;
};
} // namespace dgps