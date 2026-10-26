"""robot_state_publisher de Martha (lo reutilizan display.launch.py y la simulación).

CONCEPTOS BÁSICOS DE UN LAUNCH (aplican a todos los .launch.py del workspace):
- Un archivo launch es Python que DESCRIBE qué programas (nodos) arrancar y con qué parámetros.
  ROS llama a generate_launch_description() y ejecuta lo que devuelve.
- Node(...)                       = un programa ROS a ejecutar (paquete + ejecutable).
- DeclareLaunchArgument(...)      = un argumento que se puede pasar desde afuera:
                                    ros2 launch paquete archivo.launch.py nombre:=valor
- LaunchConfiguration('nombre')   = "el valor que tenga ese argumento". No es un string todavía:
                                    se resuelve recién cuando el launch se ejecuta.
- PathJoinSubstitution / FindPackageShare = construir rutas a archivos instalados de un paquete
                                    (install/<paquete>/share/<paquete>/...).
- Command([...])                  = ejecutar un comando de terminal y usar lo que imprime.

ROBOT_STATE_PUBLISHER: lee el URDF (la descripción del robot: piezas, medidas, articulaciones)
y publica:
  /robot_description  -> el URDF como texto (lo usan RViz y Gazebo)
  /tf, /tf_static     -> dónde está cada pieza respecto a las otras (las transformadas, "TF")
Para las articulaciones móviles (las ruedas con drive:=mecanum) necesita sus ángulos, que llegan
por /joint_states.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # use_sim_time: true = usar el reloj de Gazebo (/clock) en vez del reloj del computador.
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Ruta al URDF: install/andesrobot_description/share/andesrobot_description/urdf/...
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('andesrobot_description'), 'urdf', 'martha.urdf.xacro'])
    # Ejecutar "xacro martha.urdf.xacro sim:=... drive:=..." : xacro expande las macros y
    # devuelve el URDF final como texto. ParameterValue(..., value_type=str) le dice a ROS que
    # el resultado es un string (si no, intentaría interpretarlo como YAML y fallaría).
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' sim:=', LaunchConfiguration('sim'),
                 ' drive:=', LaunchConfiguration('drive'),
                 ' tower:=', LaunchConfiguration('tower'),
                 ' lidar_samples:=', LaunchConfiguration('lidar_samples')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='true = agrega los plugins de Gazebo (martha.sim.xacro)'),
        DeclareLaunchArgument('drive', default_value='planar', choices=['planar', 'mecanum'],
                              description='planar = planar_move (rápido); mecanum = ruedas con rodillos'),
        DeclareLaunchArgument('tower', default_value='true',
                              description='true = con la torre de aluminio'),
        DeclareLaunchArgument('lidar_samples', default_value='360',
                              description='rayos por vuelta del lidar simulado'),
        # El nodo robot_state_publisher con el URDF generado como parámetro.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}],
        ),
    ])
