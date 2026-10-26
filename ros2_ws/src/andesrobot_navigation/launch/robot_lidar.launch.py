"""Prueba del LiDAR en Martha física: LiDAR A2M8 + modelo + filtro de la torre + RViz.

Lo corre:  ./robot.sh lidar     (no necesita la base: Martha no se mueve)

Sirve para revisar, antes de mapear o navegar, que:
  - el scan llega:  ./robot.sh shell  y  ros2 topic hz /nav/scan   -> ~10 Hz
  - está bien orientado: en RViz, lo que está delante de Martha sale delante del modelo
  - el filtro borra los 4 postes de la torre: los puntos grises (/nav/scan, crudo) que caen sobre
    los postes NO tienen encima un punto rojo (/nav/scan_filtered). Si quedan puntos rojos en la
    torre, hay que agrandar la caja de config/filtro_torre.yaml.
RViz se dibuja desde base_footprint (el robot quieto en el centro): sin la base no hay odom ni
mapa, así que no hay otro frame fijo que usar.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('andesrobot_navigation')
    pkg_desc = FindPackageShare('andesrobot_description')

    # 1) LiDAR real -> /nav/scan.
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'lidar.launch.py'])),
        launch_arguments={'lidar_port': LaunchConfiguration('lidar_port')}.items(),
    )
    # 2) Modelo de Martha (TF base_footprint -> laser): sin él el scan no se puede dibujar.
    modelo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_desc, 'launch', 'rsp.launch.py'])),
        launch_arguments={'use_sim_time': 'false', 'tower': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('modelo')),
    )
    # 3) Filtro de la torre -> /nav/scan_filtered, el mismo de la navegación y el mapeo.
    filtro = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'filtro_torre.launch.py'])),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )
    # 4) RViz de navegación, pero con base_footprint como frame fijo (-f).
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'navigation.rviz']),
                   '-f', 'base_footprint'],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/martha_lidar',
                              description='Puerto serie del RPLIDAR (ver docker/udev)'),
        DeclareLaunchArgument('modelo', default_value='true',
                              description='robot_state_publisher con el URDF (false cuando lo '
                                          'lance el bringup de Estructura)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        lidar,
        modelo,
        filtro,
        rviz,
    ])
