from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('storage_path', default_value=(
            '/tmp/rolling_bonxai_icp_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))),
        Node(package='rolling_bonxai_ros', executable='rolling_bonxai_server_node',
             name='rolling_bonxai_icp', output='screen', parameters=[
                 PathJoinSubstitution([FindPackageShare('rolling_bonxai_ros'),
                                       'params', 'sonar_icp.yaml']),
                 {'rolling.storage.full_save_folder_path': LaunchConfiguration('storage_path')},
             ]),
    ])
