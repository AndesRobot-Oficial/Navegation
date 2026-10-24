"""Navegación AUTÓNOMA en simulación: Gazebo + Martha + navegación (Nav2) + RViz.

Lo corre:       ./sim.sh nav
Mandar a Martha: ./sim.sh ir sofa          (o "2D Goal Pose" en RViz)
Teclado encima de Nav2 (le quita el control mientras tocas teclas):  ./sim.sh teleop

Flujo de datos (nombres del contrato):
  Gazebo -> /nav/scan -> filtro_torre -> /nav/scan_filtered -> AMCL + costmaps
  Nav2 -> /nav/cmd_vel_planner --\
  teclado -> /cmd_vel_teleop -----> twist_mux -> /cmd_vel -> Gazebo (planar_move) -> /base/odom

OJO: en Gazebo, planar_move calcula /base/odom con la pose REAL de Martha en el mundo, así que
el frame "odom" coincide con el mundo de Gazebo, y el mapa de la casa también (se hizo así).
Por eso AMCL parte en la misma pose donde aparece Martha (x/y/yaw).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = FindPackageShare('andesrobot_gazebo')
    pkg_nav = FindPackageShare('andesrobot_navigation')
    pose = {'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw')}

    # 1) Gazebo + la casa + Martha (con su RViz apagado: aquí se abre el de navegación).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'sim.launch.py'])),
        launch_arguments={'gui': LaunchConfiguration('gazebo_gui'),
                          'rviz': 'false',
                          **pose}.items(),
    )
    # 2) Filtro del lidar + AMCL + Nav2 + twist_mux + waypoints. AMCL parte donde aparece Martha.
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav, 'launch', 'navigation.launch.py'])),
        launch_arguments={'map': LaunchConfiguration('map'),
                          'ubicaciones': LaunchConfiguration('ubicaciones'),
                          'use_sim_time': 'true',
                          **pose}.items(),
    )
    # 3) RViz de navegación: mapa, costmaps, ruta, partículas de AMCL y waypoints.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_nav, 'rviz', 'navigation.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/maps/casa.yaml'),
        DeclareLaunchArgument('ubicaciones', default_value='/maps/casa_lugares.yaml'),
        # Dónde aparece Martha en la casa (y dónde parte AMCL): afuera de la puerta del living,
        # mirando hacia adentro. Los mismos números que sim.launch.py.
        DeclareLaunchArgument('x', default_value='-7.2'),
        DeclareLaunchArgument('y', default_value='-1.862'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        sim,
        nav,
        rviz,
    ])
