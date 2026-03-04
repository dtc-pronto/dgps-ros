/*!
* @Authors Ethan Sanchez, Jason Hughes
* @Date March 2026
*
* @About Handle all the ros stuff
*/
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rtcm_msgs/msg/message.hpp>
#include <dgps_msgs/msg/nav_sat_fix_with_heading.hpp>
#include <dgps_msgs/msg/nav_sat_fix_with_quaternion.hpp>

#include "dgps/differential_gps.hpp"

namespace dgps
{
class DGPSNode : public rclcpp::Node 
{
    public:
        DGPSNode();

    private: 
              
        double baseline_;

        void publishGPS(dgps::GlobalCoord nmea);
        void publishHeading(dgps::Orientation attitude);
        void publishDiffGPS(dgps::DiffNavSatFix dgps);

        void rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg);

        std::unique_ptr<DifferentialGPS> dgps_;

        // fix publishers
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr right_fix_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr left_fix_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr avg_fix_pub_;
       
        // orientation publishers
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
        rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr orient_pub_;

        rclcpp::Publisher<dgps_msgs::msg::NavSatFixWithHeading>::SharedPtr fix_heading_pub_;
        rclcpp::Publisher<dgps_msgs::msg::NavSatFixWithQuaternion>::SharedPtr fix_quat_pub_;
        rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;
};
}
