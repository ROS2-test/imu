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

    yaw_topic_arg = DeclareLaunchArgument(
        'yaw_topic',
        default_value='imu/yaw',
        description='Yaw (rad) from IMU orientation quaternion'
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
        output='screen',
        respawn=True,
        respawn_delay=2.0,
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

    # Heading: rotate IMU quat by mount RPY (roll=π, pitch=-π/2), then yaw.
    imu_yaw_node = Node(
        package='imu',
        executable='imu_yaw_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'imu_topic': LaunchConfiguration('topic_name'),
            'yaw_topic': LaunchConfiguration('yaw_topic'),
            'use_mount': True,
            'mount_roll': 3.141592653589793,
            'mount_pitch': -1.5707963267948966,
            'mount_yaw': 0.0,
        }],
    )

    return LaunchDescription([
        port_name_arg,
        topic_name_arg,
        yaw_topic_arg,
        rate_arg,
        use_compass_arg,
        imu_node,
        imu_yaw_node,
    ])
