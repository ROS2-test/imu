import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    port_name_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyIMU',
        description='Serial port for Waveshare IMU (CH340); use /dev/ttyIMU from udev'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='imu/data_raw',
        description='Topic name for IMU data publishing'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='100',
        description='Rate in Hz for IMU data acquisition and publishing (1-250Hz)'
    )

    use_compass_arg = DeclareLaunchArgument(
        'use_compass',
        default_value='false',
        description='Mag fusion on module; leave false when EKF uses gyro ωz only'
    )

    # Define the IMU node with parameters and remappings
    imu_node = Node(
        package='imu',
        executable='imu_node',
        parameters=[{
            'port_name': LaunchConfiguration('serial_port'),
            'rate': LaunchConfiguration('rate'),
            'use_compass': LaunchConfiguration('use_compass'),
            'still_to_zero': 0,
        }],
        remappings=[
            ('imu/data', LaunchConfiguration('topic_name'))
        ]
    )

    # Return the launch description with all defined actions
    return LaunchDescription([
        port_name_arg,
        topic_name_arg,
        rate_arg,
        use_compass_arg,
        imu_node
    ]) 