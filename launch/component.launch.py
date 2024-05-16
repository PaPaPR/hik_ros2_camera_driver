import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('hik_ros2_camera_driver'), 'config', 'camera_params.yaml')

    camera_info_path = 'package://hik_ros2_camera_driver/config/camera_info.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='camera_info_path',
                              default_value=camera_info_path),
        ComposableNodeContainer(
            name='camera_nodes',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='hik_ros2_camera_driver',
                    plugin='HikCameraNode',
                    name='hik_camera',
                    extra_arguments=[
                      LaunchConfiguration('params_file'), 
                      {
                          'camera_info_path': LaunchConfiguration('camera_info_path'),
                      }
                    ],
                ),
            ]
        )
    ])