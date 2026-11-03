"""safety_filter_node: cmd_vel_in (default /nav/cmd_vel_mux) + /nav/scan -> cmd_vel_out.

Modo navegación (default): config:=safety_nav.yaml, entrada /nav/cmd_vel_mux (sale de twist_mux).
Modo teclado: config:=safety.yaml cmd_vel_in:=/cmd_vel_teleop.

Simulación y robot real publican en /cmd_vel: lo recibe planar_move o la base real.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan', default_value='/nav/scan'),
        # De dónde lee los comandos y a dónde manda los ya revisados (nombres del contrato).
        DeclareLaunchArgument('cmd_vel_in', default_value='/nav/cmd_vel_mux'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel'),
        DeclareLaunchArgument('config', default_value='safety_nav.yaml'),
        Node(
            package='andesrobot_safety',
            executable='safety_filter_node',
            name='safety_filter_node',
            output='screen',
            parameters=[PathJoinSubstitution([FindPackageShare('andesrobot_safety'),
                                              'config', LaunchConfiguration('config')]),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('scan', LaunchConfiguration('scan')),
                        ('cmd_vel_in', LaunchConfiguration('cmd_vel_in')),
                        ('cmd_vel', LaunchConfiguration('cmd_vel_out'))],
        ),
    ])
