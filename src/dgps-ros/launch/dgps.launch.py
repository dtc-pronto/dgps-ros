"""
    Jason Hughes
    March 2026

    launch dgps as an independent node
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    utm_zone_arg = DeclareLaunchArgument(
        'zone',
        default_value='18S',
        description='UTM Zone'
    )
   
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value='/dev/ttyACM0',
        description='GPS Serial Device Path'
    )

    angle_arg = DeclareLaunchArgument(
        'angle',
        default_value="-90.0"
        description="Required rotation about z-axis to align antennas with body" 
    )

    baseline_arg = DeclareLaunchArgument(
        'baseline',
        default_value="0.5",
        description="Distance between antennas in meters"
    )

    utm_zone = LaunchConfiguration('zone')
    dev = LaunchConfiguration('dev')
    angle = LaunchConfiguration('angle')
    baseline = LaunchConfiguration('baseline')
    
    dgps_node = Node(
        package='dgps',
        executable='dgps_node',
        name='dgps_node',
        output='screen',
        parameters=[
            {'utm_zone': utm_zone,
             'dev', dev,
             'angle', angle,
             'baseline', baseline
             }
        ],
    )
    
    return LaunchDescription([utm_zone_arg, 
                              dev_arg,
                              angle_arg,
                              baseline_arg,
                              dgps_node])
