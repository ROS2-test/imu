from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu',
            executable='imu_node',
            name='imu',
            output='screen',
            parameters=[{
                'port': '/dev/imu',
                'frame_id': 'imu_link',
                'baudrate': 9600,
                'device_type': 0,
                # Waveshare 10 DOF ROS IMU (A), report_tag 0x0026 — AHRS without compass
                'use_compass': False,
            }],
        ),
        Node(
            package='imu',
            executable='imu_yaw_node',
            name='imu_yaw_node',
            output='screen',
            parameters=[{
                'imu_topic': '/imu/data',
                'odom_topic': '/wheel_odom',
                'yaw_topic': '/imu/yaw',

                'freeze_wz_eps': 0.10,
                'freeze_consec': 3,
            }],
        ),
    ])
