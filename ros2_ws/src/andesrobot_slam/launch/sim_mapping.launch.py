"""MAPEO en simulación: Gazebo (casa RoboCup@Home) + Martha + slam_toolbox + RViz.

Lo corre:          ./sim.sh mapear
Manejar a Martha:  ./sim.sh teleop        (otra terminal; publica directo en /cmd_vel)
Guardar el mapa:   ./sim.sh mapa <nombre> (otra terminal; queda en martha_ws/maps/)

Para mapear bien: andar despacio, girar poco a poco y volver por lugares ya vistos (así
slam_toolbox "cierra el lazo" y el mapa cuadra). En RViz se ve crecer el mapa.

El mapa nace donde arranca slam_toolbox. En Gazebo, planar_move publica la odometría con la pose
real del mundo, así que el mapa queda alineado con el mundo, como maps/casa.yaml.

Martha parte ADENTRO del living (x=-6.3), no afuera de la puerta como en ./sim.sh (x=-7.2): afuera
de la casa el lidar no ve nada a menos de 8 m y slam_toolbox no marca esa zona, así que el punto de
partida de afuera quedaría fuera del mapa y después Nav2 no podría planificar desde ahí.
./sim.sh nav <mapa> parte en este mismo punto con los mapas que no son casa.
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
    pkg_gazebo = FindPackageShare('andesrobot_gazebo')

    # 1) Gazebo + la casa + Martha (con su RViz apagado: aquí se abre el de mapeo).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'sim.launch.py'])),
        launch_arguments={'gui': LaunchConfiguration('gazebo_gui'),
                          'drive': LaunchConfiguration('drive'),
                          'rviz': 'false',
                          'x': LaunchConfiguration('x'),
                          'y': LaunchConfiguration('y'),
                          'yaw': LaunchConfiguration('yaw')}.items(),
    )
    # 2) Filtro de la torre + slam_toolbox, con el reloj de Gazebo.
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'mapping.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    # 3) RViz de mapeo: el mapa creciendo, el lidar y Martha.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'mapping.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('drive', default_value='planar'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # Dónde aparece Martha: en el living, pasada la puerta, mirando hacia adentro.
        DeclareLaunchArgument('x', default_value='-6.3'),
        DeclareLaunchArgument('y', default_value='-1.862'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        sim,
        mapping,
        rviz,
    ])
