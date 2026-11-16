"""Démarrer le bridge, la caméra et la visualisation web, au choix."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Associer les paramètres du YAML aux trois nœuds existants."""
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('quadruped_control'), 'config', 'robot.yaml',
            ]),
            description='Fichier YAML des paramètres des nœuds.',
        ),
        DeclareLaunchArgument(
            'use_serial', default_value='true', choices=['true', 'false'],
            description='Démarrer la liaison Arduino et la publication IMU.',
        ),
        DeclareLaunchArgument(
            'use_camera', default_value='true', choices=['true', 'false'],
            description='Démarrer la publication des images.',
        ),
        DeclareLaunchArgument(
            'use_web_video', default_value='true', choices=['true', 'false'],
            description='Démarrer la visualisation dans le navigateur.',
        ),
        # Les noms correspondent aux blocs du YAML ; les topics restent à la racine.
        Node(
            package='quadruped_control', executable='serial_bridge',
            name='serial_bridge', namespace='/',
            parameters=[params_file], output='screen',
            condition=IfCondition(LaunchConfiguration('use_serial')),
        ),
        Node(
            package='camera_ros', executable='camera_node',
            name='camera', namespace='/',
            parameters=[params_file], output='screen',
            condition=IfCondition(LaunchConfiguration('use_camera')),
        ),
        Node(
            package='web_video_server', executable='web_video_server',
            name='web_video_server', namespace='/',
            parameters=[params_file], output='screen',
            condition=IfCondition(LaunchConfiguration('use_web_video')),
        ),
    ])
