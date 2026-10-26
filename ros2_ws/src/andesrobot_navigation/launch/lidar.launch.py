"""LiDAR real de Martha (RPLIDAR A2M8): el nodo nav_drivers de la arquitectura.

Solo en el robot: en simulación /nav/scan lo publica Gazebo. Corre dentro del contenedor en modo
robot (docker/compose.robot.yaml), que es el que ve los USB del laptop.

Salida (contrato):  /nav/scan  sensor_msgs/LaserScan, frame "laser", 10 Hz

    ros2 launch andesrobot_navigation lidar.launch.py
    ros2 launch andesrobot_navigation lidar.launch.py lidar_port:=/dev/ttyUSB0   (sin reglas udev)

Primera prueba (en otra terminal):  ros2 topic hz /nav/scan  -> ~10 Hz
Para verlo en RViz hace falta también la TF base_link -> laser (robot_state_publisher con el
URDF de Martha).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('andesrobot_navigation')

    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        # El nombre del nodo tiene que coincidir con la primera línea de config/lidar_a2m8.yaml.
        name='nav_drivers',
        output='screen',
        parameters=[PathJoinSubstitution([pkg, 'config', 'lidar_a2m8.yaml']),
                    {'serial_port': LaunchConfiguration('lidar_port')}],
        # sllidar_node publica en "scan"; el contrato lo llama /nav/scan.
        remappings=[('scan', '/nav/scan')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/martha_lidar',
                              description='Puerto serie del RPLIDAR (ver docker/udev)'),
        lidar,
    ])
