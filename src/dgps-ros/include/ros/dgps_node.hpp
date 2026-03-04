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
#include <rtcm_msgs/msg/message.hpp>

#include "dgps/differential_gps.hpp"

namespace dgps
{
class DGPSNode : public rclcpp::Node 
{
    public:
        DGPSNode();

    private: 
                                                 
        void publishGPS(dgps::NavSatFix nmea);
        void publishHeading(dgps::Vector3 attitude);
        void rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg);

        std::unique_ptr<DifferentialGPS> dgps_;

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
        rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_pub_;
        rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;
};
}
