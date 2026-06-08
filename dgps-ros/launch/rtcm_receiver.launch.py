"""
    2026

    Launch the RTCM receiver: a ZeroMQ SUB that pulls the base station's RTK
    correction stream and republishes it on /rtcm for the Septentrio / DGPS
    drivers to forward to the receiver hardware.

    The defaults match the base station rtk_correction broadcaster.launch
    (ip 10.10.10.10, port 7505). Point `ip` at the base station's address.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='10.10.10.10',
        description='Base station address broadcasting RTCM over ZMQ (the broadcaster binds, this node connects)'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='7505',
        description='ZMQ port the base station broadcasts RTCM on'
    )
    topic_arg = DeclareLaunchArgument(
        'rtcm_topic',
        default_value='/rtcm',
        description='Topic to republish RTCM corrections on (consumed by the Septentrio/DGPS nodes)'
    )

    rtcm_receiver_node = Node(
        package='dgps',
        executable='rtcm_receiver_node',
        name='rtcm_receiver',
        output='screen',
        parameters=[{
            'ip':         LaunchConfiguration('ip'),
            # LaunchConfiguration yields a string; coerce to int to match the node's declared type.
            'port':       ParameterValue(LaunchConfiguration('port'), value_type=int),
            'rtcm_topic': LaunchConfiguration('rtcm_topic'),
        }],
    )

    return LaunchDescription([
        ip_arg,
        port_arg,
        topic_arg,
        rtcm_receiver_node,
    ])
