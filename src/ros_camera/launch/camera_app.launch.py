import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration('param_dir', default=os.path.join(get_package_share_directory('ros_camera'), 'param', 'camera_app.yaml'))
    
    return LaunchDescription([DeclareLaunchArgument('param_dir', default_value=param_dir), 
                              Node(package='ros_camera', executable='img_publisher', name='img_publisher', parameters=[param_dir], output='screen'),
                              Node(package='ros_camera', executable='filter1', name='filter1', parameters=[param_dir], output='screen'),
                              Node(package='ros_camera', executable='filter2', name='filter2', parameters=[param_dir], output='screen'),
                              Node(package='ros_camera', executable='camera_server', name='camera_server', parameters=[param_dir], output='screen')])