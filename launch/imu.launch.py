import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    port_name_arg = DeclareLaunchArgument(
        'port_name', 
        default_value='/dev/ttyUSB0',
        description='Serial port to connect to the IMU device'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='imu/data',
        description='Topic name for IMU data publishing'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='60',
        description='Rate in Hz for IMU data acquisition and publishing (1-250Hz)'
    )

    # Define the IMU node with parameters and remappings
    imu_node = Node(
        package='imu',
        executable='imu_node',
        parameters=[{
            'port_name': LaunchConfiguration('port_name'),
            'rate': LaunchConfiguration('rate')
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
        imu_node
    ]) 