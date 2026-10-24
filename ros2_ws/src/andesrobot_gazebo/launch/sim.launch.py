"""Martha en Gazebo Classic 11: mundo + robot_state_publisher + spawn (+ RViz).

Expone los topics del contrato, igual que el robot real:
  /nav/scan, /base/imu, /cmd_vel (entrada), /base/odom, TF odom -> base_footprint, /clock.

Orden de lo que pasa:
 1. Gazebo abre el mundo (por defecto la casa RoboCup@Home).
 2. robot_state_publisher publica el URDF en /robot_description (con los plugins de Gazebo).
 3. spawn_entity.py lee ese URDF y "pone" a Martha dentro de Gazebo.
 4. Los plugins del URDF (martha.sim.xacro) empiezan a publicar lidar, IMU y odometría.
 5. Con drive:=mecanum, además se arrancan los controladores de las ruedas (ros2_control).

Uso:  ros2 launch andesrobot_gazebo sim.launch.py            (lo que corre ./sim.sh)
      ros2 launch andesrobot_gazebo sim.launch.py gui:=false rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Carpetas instaladas de los paquetes que usamos.
    pkg_gazebo = FindPackageShare('andesrobot_gazebo')
    pkg_desc = FindPackageShare('andesrobot_description')

    # Posición inicial de Martha en el mundo (metros y radianes).
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')

    # Gazebo: incluir el launch oficial de gazebo_ros, que arranca gzserver (la física) y
    # gzclient (la ventana) y carga los plugins que conectan Gazebo con ROS.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gui'),
                          'verbose': LaunchConfiguration('verbose')}.items(),
    )

    # robot_state_publisher en modo simulación:
    #   use_sim_time=true -> usa el reloj de Gazebo
    #   sim=true          -> agrega los plugins de martha.sim.xacro (lidar, IMU, movimiento)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_desc, 'launch', 'rsp.launch.py'])),
        launch_arguments={'use_sim_time': 'true',
                          'sim': 'true',
                          'drive': LaunchConfiguration('drive'),
                          'tower': LaunchConfiguration('tower')}.items(),
    )

    # spawn_entity.py: toma el URDF del topic robot_description y crea el robot en Gazebo
    # con el nombre "martha", en (x, y), 2 cm sobre el suelo (para que caiga suave) y girado
    # "yaw" radianes.
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'martha',
                   '-x', x, '-y', y, '-z', '0.02', '-Y', yaw],
        output='screen',
    )

    # drive:=mecanum -> los controladores de ros2_control. El controller_manager recién existe
    # cuando Martha ya está en Gazebo, y los controladores se arrancan uno después del otro, por
    # eso cada uno espera a que termine el anterior (OnProcessExit).
    es_mecanum = LaunchConfigurationEquals('drive', 'mecanum')
    broadcaster = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster'], condition=es_mecanum)
    controller = Node(package='controller_manager', executable='spawner', output='screen',
                      arguments=['mecanum_drive_controller'], condition=es_mecanum)

    # RViz con lo que ve Martha: modelo, scan, odometría y TF (rviz/sim.rviz).
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', PathJoinSubstitution([pkg_gazebo, 'rviz', 'sim.rviz'])],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        # Por defecto: la casa RoboCup@Home (la misma de milo_ws).
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([pkg_gazebo, 'worlds', 'robocup_casa.world']),
            description='Ruta al .world'),
        DeclareLaunchArgument('gui', default_value='true', description='Abrir gzclient'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='Logs de Gazebo ([Err]/[Wrn]) en la terminal'),
        DeclareLaunchArgument('drive', default_value='planar', choices=['planar', 'mecanum'],
                              description='planar = planar_move (rápido); mecanum = ruedas con rodillos'),
        DeclareLaunchArgument('tower', default_value='true',
                              description='true = con la torre de aluminio'),
        # Pose inicial. Por defecto, afuera de la puerta del living mirando hacia adentro (la
        # misma partida de Milo, CASA_SPAWN en sim.sh).
        DeclareLaunchArgument('x', default_value='-7.2'),
        DeclareLaunchArgument('y', default_value='-1.862'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo,
        rsp,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=[controller])),
        rviz,
    ])
