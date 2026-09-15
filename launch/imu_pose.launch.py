import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Serial port for Waveshare IMU (CH340)'
    )
    rate_arg = DeclareLaunchArgument(
        'rate', default_value='60',
        description='IMU report rate in Hz (1-250)'
    )
    report_tag_arg = DeclareLaunchArgument(
        'report_tag', default_value='255',
        description='Subscription tag: 0x00FF = sensors + INS position (bit 0x80)'
    )

    imu_node = Node(
        package='imu',
        executable='imu_node',
        parameters=[{
            'port_name': LaunchConfiguration('port_name'),
            'rate': LaunchConfiguration('rate'),
            'report_tag': LaunchConfiguration('report_tag'),
            'still_to_zero': 0,
            'use_compass': True,
        }]
    )

    rviz_config = os.path.join(
        get_package_share_directory('imu'), 'rviz', 'imu_pose.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        port_name_arg,
        rate_arg,
        report_tag_arg,
        imu_node,
        rviz_node,
    ])
