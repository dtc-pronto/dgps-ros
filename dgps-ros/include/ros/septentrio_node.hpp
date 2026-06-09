/*!
* @Date 2026
*
* @About ROS2 component for the Septentrio mosaic-G5 P3H receiver.
*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rtcm_msgs/msg/message.hpp>
#include <dgps_msgs/msg/differential_nav_sat_fix.hpp>

#include "dgps/septentrio.hpp"
#include "dgps/geodetics.hpp"

namespace dgps
{
class SeptentrioNode : public rclcpp::Node
{
    public:
        SeptentrioNode(const rclcpp::NodeOptions& options);

    private:
        double baseline_param_;
        double angle_;
        char utm_zone_[10];

        double transformHeading(double heading);

        void publishGPS(GlobalCoord gc);
        void publishHeading(Orientation att);
        void publishBaseline(Baseline b);
        void publishVelocity(Velocity v);
        void publishDiffGPS(DiffNavSatFix d);

        void rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg);

        std::unique_ptr<SeptentrioGPS> sept_;
        std::unique_ptr<Baseline> last_baseline_;

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr ant1_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr ant2_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr center_pub_;
        rclcpp::Publisher<dgps_msgs::msg::DifferentialNavSatFix>::SharedPtr dfix_pub_;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_deg_pub_;
        rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr orient_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;

        rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;
};
} // namespace dgps
