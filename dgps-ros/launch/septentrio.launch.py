"""
    2026

    Launch the Septentrio mosaic-G5 P3H node.
    Reads NMEA from one serial port and forwards /rtcm to a second port.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nmea_dev_arg = DeclareLaunchArgument(
        'nmea_dev',
        default_value='/dev/ttyACM0',
        description='Serial device for NMEA output stream (USB1 on the mosaic-G5)'
    )
    nmea_baud_arg = DeclareLaunchArgument(
        'nmea_baud',
        default_value='115200',
        description='Baud rate for the NMEA serial port'
    )
    rtcm_dev_arg = DeclareLaunchArgument(
        'rtcm_dev',
        default_value='/dev/ttyACM1',
        description='Serial device for RTCM input forwarding (USB2 on the mosaic-G5). Set empty to disable.'
    )
    rtcm_baud_arg = DeclareLaunchArgument(
        'rtcm_baud',
        default_value='115200',
        description='Baud rate for the RTCM serial port (ignored on USB-CDC ports)'
    )
    baseline_arg = DeclareLaunchArgument(
        'baseline',
        default_value='0.5',
        description='Antenna separation in meters (fallback when RBP unavailable)'
    )
    angle_arg = DeclareLaunchArgument(
        'angle',
        default_value='90.0',
        description='Rotation [deg] about z-axis to align the antenna baseline with the vehicle forward axis'
    )
    utm_zone_arg = DeclareLaunchArgument(
        'zone',
        default_value='18S',
        description='UTM zone'
    )

    septentrio_node = Node(
        package='dgps',
        executable='septentrio_node',
        name='septentrio_node',
        output='screen',
        parameters=[{
            'nmea_dev':  LaunchConfiguration('nmea_dev'),
            'nmea_baud': LaunchConfiguration('nmea_baud'),
            'rtcm_dev':  LaunchConfiguration('rtcm_dev'),
            'rtcm_baud': LaunchConfiguration('rtcm_baud'),
            'baseline':  LaunchConfiguration('baseline'),
            'angle':     LaunchConfiguration('angle'),
            'utm_zone':  LaunchConfiguration('zone'),
        }],
    )

    return LaunchDescription([
        nmea_dev_arg,
        nmea_baud_arg,
        rtcm_dev_arg,
        rtcm_baud_arg,
        baseline_arg,
        angle_arg,
        utm_zone_arg,
        septentrio_node,
    ])
