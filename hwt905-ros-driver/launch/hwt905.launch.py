import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('hwt905_ros_driver'),
        'config',
        'hwt905_config.yaml',
    )

    return LaunchDescription([
        Node(
            package='hwt905_ros_driver',
            executable='hwt905_ros_driver',
            name='hwt905_ros_driver',
            parameters=[config],
            output='screen',
        ),
    ])
