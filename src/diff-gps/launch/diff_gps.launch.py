from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Launch arguments
    test_arg = DeclareLaunchArgument(
        'test',
        default_value='0.0',
        description='Test parameter for diff_gps'
    )

    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='0.0.0.0',
        description='RTK broadcaster IP'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='2101',
        description='RTK broadcaster port'
    )

    # diff_gps node
    gps_node = Node(
        package='diff_gps',
        executable='diff_gps_node',
        name='diff_gps_node',
        output='screen',
        parameters=[{
            'test': LaunchConfiguration('test')
        }]
    )

    # rtk_correction receiver node
    rtk_receiver_node = Node(
        package='rtk_correction',
        executable='receiver',
        name='rtk_receiver_node',
        output='screen',
        parameters=[{
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )
    
    rtk_broadcaster_node = Node(
        package='rtk_correction',
        executable='broadcaster',
        name='rtk_broadcaster_node',
        output='screen',
        parameters=[{
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )

    return LaunchDescription([
        test_arg,
        ip_arg,
        port_arg,
        gps_node,
        rtk_receiver_node,
        rtk_broadcaster_node
    ])
