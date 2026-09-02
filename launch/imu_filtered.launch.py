import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    port_name_arg = DeclareLaunchArgument(
        'serial_port', 
        default_value='/dev/ttyUSB2',
        description='Serial port to connect to the IMU device'
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

    use_filter_arg = DeclareLaunchArgument(
        'use_imu_filter',
        default_value='true',
        description='Enable Madgwick filter'
    )

    # IMU драйвер
    imu_node = Node(
        package='imu',
        executable='imu_node',
        parameters=[{
            'port_name': LaunchConfiguration('serial_port'),
            'rate': LaunchConfiguration('rate')
        }],
        remappings=[
            ('imu/data', LaunchConfiguration('topic_name'))
        ]
    )

    # Madgwick filter
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_imu_filter')),
        remappings=[
            ('imu/data_raw', LaunchConfiguration('topic_name')),
        ],
        parameters=[{
                'use_mag': False,  # Использовать магнитометр
                'gain': 0.1,  # Коэффициент фильтра
                'zeta': 0.0,  # Коэффициент для магнитометра (если use_mag=true)
                'use_adaptive_gain': False, # Использовать адаптивный gain
                'world_frame': 'enu',  # Система координат: 'enu' (East-North-Up) или 'ned' (North-East-Down)
                'publish_tf': False,  # Публиковать ли TF
                
                # --- Параметры TF (используются, если publish_tf=true) ---
                # 'fixed_frame': 'odom',   # Родительский фрейм для TF (по умолчанию 'odom')
                # 'reverse_tf': False,      # Инвертировать TF (из imu_frame в fixed_frame)

                # --- Дополнительные параметры ---
                # 'remove_gravity_vector': False, # Удалить вектор гравитации из акселерометра
                # 'constant_dt': 0.0,      # Постоянный временной ша
            }]
    )

    return LaunchDescription([
        port_name_arg,
        topic_name_arg,
        rate_arg,
        use_filter_arg,
        imu_node,
        imu_filter_node,
    ])