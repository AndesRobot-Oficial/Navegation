"""MAPEO en Martha física: LiDAR A2M8 + modelo de Martha + slam_toolbox (+ RViz).

Lo corre:          ./robot.sh mapear      (en el laptop de Martha)
Manejar a Martha:  ./robot.sh teleop      (otra terminal)
Guardar el mapa:   ./robot.sh mapa <nombre>

Lo que pone Navegación: el LiDAR (nav_drivers -> /nav/scan), el filtro de la torre y
slam_toolbox. Lo que tiene que estar corriendo de ESTRUCTURA (la base): /base/odom con la TF
odom -> base_footprint, y alguien que escuche /cmd_vel. Sin eso slam_toolbox no recibe la
odometría y no arma nada (en la terminal se queja de que no encuentra la TF odom).

modelo:=true levanta robot_state_publisher con el URDF de Martha (TF base_footprint -> laser y
el resto del robot). En la arquitectura eso es de Estructura; mientras su bringup no exista lo
lanzamos aquí. Cuando exista: modelo:=false (si no, las TF fijas salen dos veces).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('andesrobot_slam')
    pkg_nav = FindPackageShare('andesrobot_navigation')
    pkg_desc = FindPackageShare('andesrobot_description')

    # 1) LiDAR real -> /nav/scan (andesrobot_navigation/launch/lidar.launch.py).
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav, 'launch', 'lidar.launch.py'])),
        launch_arguments={'lidar_port': LaunchConfiguration('lidar_port')}.items(),
    )
    # 2) Modelo de Martha como está construida (con la torre), con el reloj del computador.
    modelo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_desc, 'launch', 'rsp.launch.py'])),
        launch_arguments={'use_sim_time': 'false', 'tower': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('modelo')),
    )
    # 3) Filtro de la torre + slam_toolbox, con el reloj del computador (no hay Gazebo).
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'mapping.launch.py'])),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )
    # 4) RViz de mapeo (solo si el laptop tiene pantalla; ./robot.sh lo decide).
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'mapping.rviz'])],
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
        mapping,
        rviz,
    ])
