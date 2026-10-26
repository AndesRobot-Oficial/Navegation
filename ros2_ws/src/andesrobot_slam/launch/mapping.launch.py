"""Modo MAPEO de Martha (slam_node de la arquitectura): filtro de la torre + slam_toolbox.

Sirve igual para simulación (use_sim_time:=true) y para Martha física (false). NO levanta el
robot, el lidar ni Gazebo: eso lo hacen sim_mapping.launch.py y robot_mapping.launch.py.

Flujo de datos (nombres del contrato):
  /nav/scan --> filtro_torre (borra los postes de la torre) --> /nav/scan_filtered
  /nav/scan_filtered + odometría (TF odom -> base_footprint) --> slam_toolbox
      --> /nav/map (el mapa que se va armando) + TF map -> odom

SLAM = Simultaneous Localization And Mapping: arma el mapa y al mismo tiempo calcula dónde está
el robot dentro de ese mapa. "online async" = procesa los scans en vivo y, si llega uno mientras
procesa otro, se salta el nuevo (no se atrasa).

En modo COMPETENCIA no corre esto: el mapa ya está hecho y lo publica map_server, y AMCL ubica a
Martha en él (andesrobot_navigation/launch/navigation.launch.py). Los dos publican /nav/map y la
TF map -> odom, así que nunca hay que lanzarlos a la vez.

Guardar el mapa (otra terminal):  ./sim.sh mapa <nombre>  o  ./robot.sh mapa <nombre>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('andesrobot_slam')
    pkg_nav = FindPackageShare('andesrobot_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Filtro del lidar: el mismo de la navegación (andesrobot_navigation/filtro_torre.launch.py),
    # para que el mapa no tenga "obstáculos" encima del robot.
    filtro = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav, 'launch', 'filtro_torre.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # slam_toolbox asíncrono con los parámetros de config/slam_toolbox_mapping.yaml.
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params'), {'use_sim_time': use_sim_time}],
        # slam_toolbox publica el mapa en /map; el contrato lo llama /nav/map.
        remappings=[('/map', '/nav/map'), ('/map_metadata', '/nav/map_metadata')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=PathJoinSubstitution([pkg, 'config', 'slam_toolbox_mapping.yaml'])),
        filtro,
        slam,
    ])
