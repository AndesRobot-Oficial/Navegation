"""Navegación AUTÓNOMA en Martha física: LiDAR A2M8 + modelo + navegación (Nav2) (+ RViz).

Lo corre:          ./robot.sh nav <mapa>   (en el laptop de Martha, con un mapa de maps/)
Mandar a Martha:   ./robot.sh ir <lugar>   (o "2D Goal Pose" en RViz)
Teclado encima de Nav2 (le quita el control mientras tocas teclas):  ./robot.sh teleop

Es lo mismo que sim_navigation.launch.py cambiando la capa de abajo: el LiDAR real en vez del
de Gazebo, y el reloj del computador (use_sim_time:=false) en vez del de Gazebo.

Lo que tiene que estar corriendo de ESTRUCTURA (la base): /base/odom con la TF
odom -> base_footprint, y alguien que escuche /cmd_vel. modelo:=true levanta aquí
robot_state_publisher mientras el bringup de Estructura no exista (ver robot_mapping.launch.py
en andesrobot_slam).

Pose inicial: los mapas reales tienen el origen donde empezó el mapeo, así que por defecto AMCL
cree que Martha está ahí (x=0 y=0 yaw=0). Si la pusiste en otro lado: "2D Pose Estimate" en RViz.
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
    # 2) Modelo de Martha como está construida (con la torre), con el reloj del computador.
    modelo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_desc, 'launch', 'rsp.launch.py'])),
        launch_arguments={'use_sim_time': 'false', 'tower': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('modelo')),
    )
    # 3) Filtro del lidar + AMCL + Nav2 + twist_mux + waypoints, igual que en la simulación.
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'navigation.launch.py'])),
        launch_arguments={'map': LaunchConfiguration('map'),
                          'ubicaciones': LaunchConfiguration('ubicaciones'),
                          'use_sim_time': 'false',
                          'x': LaunchConfiguration('x'),
                          'y': LaunchConfiguration('y'),
                          'yaw': LaunchConfiguration('yaw')}.items(),
    )
    # 4) RViz de navegación (solo si el laptop tiene pantalla; ./robot.sh lo decide).
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'navigation.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Ruta al .yaml del mapa (/maps/<mapa>.yaml)'),
        # Si el archivo todavía no existe (mapa recién hecho), arranca sin waypoints.
        DeclareLaunchArgument('ubicaciones',
                              description='Waypoints del mapa (/maps/<mapa>_lugares.yaml)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/martha_lidar',
                              description='Puerto serie del RPLIDAR (ver docker/udev)'),
        DeclareLaunchArgument('modelo', default_value='true',
                              description='robot_state_publisher con el URDF (false cuando lo '
                                          'lance el bringup de Estructura)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # Pose inicial en el mapa (con punto decimal). 0 = donde empezó el mapeo.
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        lidar,
        modelo,
        nav,
        rviz,
    ])
