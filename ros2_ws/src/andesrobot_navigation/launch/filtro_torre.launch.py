"""Filtro del lidar de Martha: /nav/scan -> /nav/scan_filtered, sin los postes de la torre.

Lo incluyen la navegación (navigation.launch.py), el mapeo (andesrobot_slam/mapping.launch.py) y
la prueba del LiDAR real (robot_lidar.launch.py): así los tres limpian el scan igual. Qué borra y
por qué: config/filtro_torre.yaml.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('andesrobot_navigation')

    filtro = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        # El nombre del nodo tiene que coincidir con la primera línea de config/filtro_torre.yaml.
        name='filtro_torre',
        output='screen',
        parameters=[PathJoinSubstitution([pkg, 'config', 'filtro_torre.yaml']),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('scan', '/nav/scan'), ('scan_filtered', '/nav/scan_filtered')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        filtro,
    ])
