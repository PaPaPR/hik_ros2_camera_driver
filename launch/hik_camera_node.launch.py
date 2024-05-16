import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('hik_ros2_camera_driver'), 'config', 'camera_params.yaml')

    camera_info_path = 'package://hik_ros2_camera_driver/config/camera_info.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='camera_info_path',
                              default_value=camera_info_path),
        Node(
            package='hik_ros2_camera_driver',
            name='hik_camera',
            executable='hik_camera_node',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('hik_camera', 'image'),
            ],
            parameters=[
                LaunchConfiguration('params_file'), 
                {
                    'camera_info_path': LaunchConfiguration('camera_info_path'),
                }
            ],
        ),
    ])