"""Muestra a Martha en RViz, sin Gazebo.

Uso:  ros2 launch andesrobot_description display.launch.py
      ros2 launch andesrobot_description display.launch.py drive:=mecanum tower:=false
Sirve para revisar el modelo (URDF): medidas, frames (base_footprint, base_link, laser,
imu_link) y la torre.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Carpeta instalada de este paquete (para encontrar launch/ y rviz/).
    pkg = FindPackageShare('andesrobot_description')

    # robot_state_publisher sin los plugins de Gazebo (sim:=false).
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'rsp.launch.py'])),
        launch_arguments={'use_sim_time': 'false',
                          'drive': LaunchConfiguration('drive'),
                          'tower': LaunchConfiguration('tower')}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('drive', default_value='planar', choices=['planar', 'mecanum']),
        DeclareLaunchArgument('tower', default_value='true'),
        rsp,
        # Con drive:=mecanum las ruedas son articulaciones: joint_state_publisher las publica
        # todas en 0 para que RViz las pueda dibujar. Con planar no hace nada (no hay ninguna).
        Node(package='joint_state_publisher', executable='joint_state_publisher'),
        # RViz con la configuración guardada en rviz/display.rviz ("-d" = cargar ese archivo).
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'display.rviz'])]),
    ])
