"""Navegación autónoma de Martha: filtro del lidar + map_server + AMCL + Nav2 + twist_mux.

Sirve igual para simulación (use_sim_time:=true) y para Martha física (false). NO levanta el
robot ni Gazebo: eso lo hace sim_navigation.launch.py (en la simulación).

Entradas (contrato):  /nav/scan, /base/odom, TF odom -> base_footprint, un mapa .yaml (map:=)
Salidas (contrato):   /cmd_vel, /nav/map, TF map -> odom, acción /nav/navigate_to_pose
Internos de Navegación: /nav/scan_filtered, /nav/cmd_vel_raw, /nav/cmd_vel_planner, costmaps,
                       /plan, /nav/ubicaciones (marcadores de los waypoints en RViz)

controller:=ppo (por defecto) o dwb elige quién decide las velocidades al seguir la ruta:
  ppo  la política de la tesis de Nicolás (ppo_controller.py, atiende /follow_path)
  dwb  el controlador clásico de Nav2, de respaldo y para comparar

Qué es cada nodo de la arquitectura:
  localization_node = map_server + amcl (+ filtro_torre, que limpia el scan)
  path_planner_node = planner_server + controller_server + smoother + behaviors + bt_navigator
                      + velocity_smoother (atiende /nav/navigate_to_pose)
  cmd_vel_mux       = twist_mux

Se escribe cada nodo a mano (en vez de incluir nav2_bringup) para poner los nombres del contrato
con remaps: en nav2_bringup (Humble) los tópicos son /map, /cmd_vel y /navigate_to_pose.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('andesrobot_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('nav_params')
    autostart = LaunchConfiguration('autostart')

    # Lo que comparten todos los nodos de Nav2: el yaml + el reloj.
    common = [params, {'use_sim_time': use_sim_time}]
    # Todos los nodos corren en el namespace raíz y usan las TF globales.
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ---------------- Filtro del lidar ----------------
    # Saca del scan los postes de la torre (ver config/filtro_torre.yaml).
    filtro = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='filtro_torre',
        output='screen',
        parameters=[PathJoinSubstitution([pkg, 'config', 'filtro_torre.yaml']),
                    {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/nav/scan'), ('scan_filtered', '/nav/scan_filtered')],
    )

    # ---------------- Localización ----------------
    # map_server lee el .pgm + .yaml y lo publica en /nav/map (latched: lo recibe quien llegue
    # después). AMCL compara el scan con ese mapa y publica la TF map -> odom.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=common + [{'yaml_filename': LaunchConfiguration('map')}],
        remappings=tf + [('map', '/nav/map')],
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        # Pose inicial: la que pase el launch (x:= y:= yaw:=), con punto decimal.
        parameters=common + [{
            'initial_pose.x': ParameterValue(LaunchConfiguration('x'), value_type=float),
            'initial_pose.y': ParameterValue(LaunchConfiguration('y'), value_type=float),
            'initial_pose.yaw': ParameterValue(LaunchConfiguration('yaw'), value_type=float),
        }],
        remappings=tf,
    )
    # El lifecycle_manager activa los nodos en orden (configurar -> activar).
    lm_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': ['map_server', 'amcl']}],
    )

    # ---------------- Navegación ----------------
    # Con el PPO, controller_server sigue corriendo porque mantiene el costmap local que usan las
    # recuperaciones de Nav2 (girar, retroceder), pero con su acción renombrada para que nadie le
    # pida seguir la ruta. Como en /nav/navigate_to_pose, se remapean sus 5 partes.
    dwb_en_espera = [(f'follow_path/_action/{p}', f'dwb_en_espera/_action/{p}')
                     for p in ('send_goal', 'cancel_goal', 'get_result', 'feedback', 'status')]
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=common,
        remappings=tf + [('cmd_vel', '/nav/cmd_vel_raw')] + dwb_en_espera,
        condition=LaunchConfigurationEquals('controller', 'ppo'),
    )
    # Con DWB: controller_server atiende /follow_path, como siempre.
    controller_dwb = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=common,
        remappings=tf + [('cmd_vel', '/nav/cmd_vel_raw')],
        condition=LaunchConfigurationEquals('controller', 'dwb'),
    )
    # El PPO: atiende /follow_path en vez de controller_server. Lee el scan sin filtrar porque se
    # entrenó viendo los postes de la torre, y su salida no pasa por velocity_smoother.
    ppo = Node(
        package='andesrobot_navigation',
        executable='ppo_controller',
        name='ppo_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'politica': LaunchConfiguration('politica')}],
        remappings=tf + [('scan', '/nav/scan'), ('odom', '/base/odom'),
                         ('cmd_vel', '/nav/cmd_vel_planner')],
        condition=LaunchConfigurationEquals('controller', 'ppo'),
    )
    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=common,
        remappings=tf,
    )
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=common,
        remappings=tf,
    )
    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=common,
        remappings=tf + [('cmd_vel', '/nav/cmd_vel_planner')],
    )
    # bt_navigator atiende la acción que usa Interacción: /nav/navigate_to_pose (contrato).
    # En Humble una acción no se puede remapear por su nombre: hay que remapear los 3 servicios
    # y 2 tópicos con que está hecha por dentro (en ROS 2 una acción es eso).
    accion = [(f'navigate_to_pose/_action/{p}', f'/nav/navigate_to_pose/_action/{p}')
              for p in ('send_goal', 'cancel_goal', 'get_result', 'feedback', 'status')]
    bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=common,
        remappings=tf + accion,
    )
    vel_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=common,
        remappings=tf + [('cmd_vel', '/nav/cmd_vel_raw'),
                         ('cmd_vel_smoothed', '/nav/cmd_vel_planner')],
    )
    lm_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': ['controller_server', 'smoother_server', 'planner_server',
                                    'behavior_server', 'bt_navigator', 'velocity_smoother']}],
    )

    # ---------------- cmd_vel_mux: teclado > mirar > navegar ----------------
    mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[PathJoinSubstitution([pkg, 'config', 'twist_mux.yaml']),
                    {'use_sim_time': use_sim_time}],
        # El filtro de seguridad aún no está conectado (NAV-10): publica en /cmd_vel.
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    # ---------------- Girar buscando (/nav/search_rotate) ----------------
    # Gira en el lugar, despacio, mientras Visión busca un objeto. Publica en /nav/cmd_vel_look.
    girar = Node(
        package='andesrobot_navigation',
        executable='girar_buscando',
        name='girar_buscando',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---------------- Waypoints en RViz ----------------
    markers = Node(
        package='andesrobot_navigation',
        executable='marcadores',
        name='ubicaciones_rviz',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'ubicaciones': LaunchConfiguration('ubicaciones')}],
    )

    return LaunchDescription([
        # Los mapas viven en martha_ws/maps/ (tu PC), que compose.yaml monta como /maps.
        DeclareLaunchArgument('map', default_value='/maps/casa.yaml',
                              description='Ruta al .yaml del mapa'),
        DeclareLaunchArgument('ubicaciones', default_value='/maps/casa_lugares.yaml',
                              description='Waypoints con nombre (x, y, yaw en el frame map)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        # Se llama nav_params y no params_file (el nombre de nav2_bringup) porque gazebo.launch.py
        # declara su propio params_file vacío y, en la simulación, pisaría a este.
        DeclareLaunchArgument(
            'nav_params',
            default_value=PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml'])),
        # Pose inicial del robot en el mapa (con punto decimal).
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('controller', default_value='ppo', choices=['ppo', 'dwb'],
                              description='Quién decide las velocidades al seguir la ruta'),
        # policies/ de tu PC, que compose.yaml monta como /policies (igual que maps/).
        DeclareLaunchArgument('politica', default_value='/policies/martha_holo.npz',
                              description='La política exportada de la tesis (.npz)'),
        filtro,
        map_server,
        amcl,
        lm_loc,
        controller,
        controller_dwb,
        ppo,
        smoother,
        planner,
        behaviors,
        bt,
        vel_smoother,
        lm_nav,
        mux,
        girar,
        markers,
    ])
