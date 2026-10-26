# Arquitectura del robot y contrato de integración

Resumen de <https://alejapinzonf.github.io/Robot_arquitecture_ROS2/>, leído el **2026-10-14** y
revisado de nuevo el **2026-10-24** (la página no había cambiado: el contrato tiene las mismas 58
interfaces).
La página es la fuente de verdad: si algo de aquí no coincide con ella, manda la página (y hay que
actualizar este archivo). La página tiene además animaciones del reto, el grafo interactivo de nodos
y una vista 3D (`vista3d.html`) que aquí no se copian.

## Equipos y prefijos

Cinco equipos, cada uno con su prefijo de nombres. Cada equipo implementa sus nodos como quiera y
solo expone las interfaces del contrato. **Una interfaz tiene un único responsable** (regla del
proyecto, no de ROS 2).

| Equipo | Prefijo | Qué hace |
|---|---|---|
| Interacción Humano-Robot (HRI) | `/hri/` | Orquestador del reto (`task_orchestrator_node`): decide y pide. Voz, diálogo, LLM, micrófono y parlante. |
| Visión Computacional | `/vision/` | Personas, caras, gestos, objetos, asientos, chapa. Configura las cámaras. |
| **Navegación** (nosotros) | `/nav/` | Mapa, localización, rutas, seguir y mirar personas, puerta. Configura el **LiDAR**. |
| Brazo y Garra | `/arm/` | Brazo, garra, ascensor, abrir la puerta, handover. |
| **Estructura y Electrónica** (nosotros) | `/base/` | Motores, encoders, **IMU**, odometría (EKF), botones, emergencia, luces, `robot_state_publisher`. |

Regla de los sensores: **cada sensor lo configura el equipo que lo usa**. LiDAR → Navegación,
IMU → Estructura (de ella sale la odometría), cámaras → Visión, micrófono y parlante → HRI.

## La frontera Navegación / Estructura

**Navegación nunca toca un motor.** Solo cruzan dos cosas:

- hacia abajo `/cmd_vel` (`geometry_msgs/Twist`, en `base_footprint`, m/s y rad/s, nunca PWM);
- hacia arriba `/base/odom` (`nav_msgs/Odometry`) y la TF `odom -> base_footprint`.

Navegación no le dice a Estructura "anda a (x, y)": eso es la acción `/nav/navigate_to_pose`, que
es de Navegación. A Estructura solo le llega velocidad. **Si `/cmd_vel` deja de llegar 0,5 s, la
base se detiene** (watchdog de `base_driver_node`).

### Cadena de velocidad (dentro de Navegación)

```
path_planner_node  --/nav/cmd_vel_planner-->  cmd_vel_mux  --/nav/cmd_vel_mux-->  safety_filter_node  --/cmd_vel-->  base
look_controller_node --/nav/cmd_vel_look--^   (twist_mux)                         (frena a < 30 cm)
```

- El **único** publicador de `/cmd_vel` es `safety_filter_node`. El planificador local publica
  `/nav/cmd_vel_planner`, no `/cmd_vel`.
- El mux es obligatorio porque mirar a una persona es girar la base. Mirar gana cuando el robot está
  quieto conversando.
- El filtro publica su estado en `/nav/safety_state` (`std_msgs/String`). **En MARTHA tiene que mirar
  también a los lados**, porque se mueve de lado.
- En la prueba no hay teleop: el launch de competencia ni lo lanza. El teleop (`/cmd_vel_teleop`)
  es solo para probar y mapear en los días de preparación.

### Odometría (dentro de Estructura)

```
base_driver_node --/base/wheel_odom--> odom_ekf_node --/base/odom + TF odom->base_footprint-->
imu_driver_node  --/base/imu---------^  (robot_localization/ekf_node + yaml)
```

- `base_driver_node`: ros2_control con un hardware interface propio para los 4 motores y un
  controlador omnidireccional. En MARTHA `/base/wheel_odom` lleva también `twist.linear.y`.
- `odom_ekf_node` es el `ekf_node` de `robot_localization` en modo 2D; es **la única fuente de la TF
  `odom -> base_footprint`** (el controlador de ruedas va con `enable_odom_tf: false`). En MARTHA el
  EKF estima también la velocidad lateral. Corre igual en simulación y en el robot (mismo yaml).
- Mientras el EKF no exista, la odometría de las ruedas sale directo como `/base/odom`.

### Árbol de TF

```
map --(localization_node; en mapeo slam_node)--> odom --(odom_ekf_node)--> base_footprint
base_footprint --(robot_state_publisher, URDF)--> base_link --> laser, imu_link_1, brazo...
```

## Nodos de Navegación

| Nodo | Qué hace | Interfaces |
|---|---|---|
| `nav_drivers` | LiDAR con `sllidar_node`, frame `laser` | publica `/nav/scan` |
| `slam_node` | Modo **mapeo**: construye y guarda el mapa (con teleop) | `/nav/map`, `/nav/pose`, TF `map -> odom` |
| `localization_node` | Modo **competencia**: se ubica en el mapa guardado | `/nav/map`, `/nav/pose`, TF `map -> odom`, servicio `/nav/set_initial_pose` |
| `path_planner_node` | Planifica y sigue la ruta | servidor de `/nav/navigate_to_pose`; publica `/nav/cmd_vel_planner` |
| `person_follower_node` | Sigue a una persona (0,8–1 m detrás) mandando metas a `path_planner_node` | servidor de `/nav/follow_person`; lee `/vision/persons` |
| `look_controller_node` | Gira la base para mirar a una persona o hacia donde navega | `/nav/look` (acción), `/nav/look_ahead` (servicio); publica `/nav/cmd_vel_look` |
| `door_detector_node` | Con el LiDAR: ¿puerta abierta? ¿dónde está la hoja? | servicio `/nav/detect_door` |
| `cmd_vel_mux` | `twist_mux`: elige entre navegar y mirar | `/nav/cmd_vel_mux` |
| `safety_filter_node` | Frena si hay algo a menos de 30 cm | publica `/cmd_vel`, `/nav/safety_state` |

Detalles del contrato para Navegación:

- `/nav/scan`: `header.frame_id = laser`, 10 Hz. Si hay que quitar del scan la estructura del robot,
  el scan filtrado es interno de Navegación (en milo_ws, `/scan_filtered`).
- `/nav/navigate_to_pose`: meta `PoseStamped` en `map` → feedback `distance_remaining` → resultado
  `bool success`. Tipo `NavigateToPose.action` de Nav2.
- `/nav/follow_person`: meta `int32 person_id` → feedback `distance_to_person` → `success`. Si
  pierde a la persona más de 5 s, termina con `success = false`.
- `/nav/look`: meta `person_id` (−1 = usar `target`), `PointStamped target`, `bool continuous` →
  feedback `on_target` → `success`.
- `/nav/look_ahead`: `std_srvs/SetBool` (true = mirar hacia donde navega).
- `/nav/detect_door`: petición vacía → `is_open`, `door_pose` (en `map`), `hinge_side`. La chapa no
  es de Navegación: es `/vision/detect_handle`.
- `/nav/set_initial_pose`: `PoseWithCovarianceStamped` en `map` → `success`, `message`. Por dentro
  se reenvía a slam_toolbox o AMCL. Si no converge en 3 s, `success = false`.
- `/nav/pose`: `PoseStamped` en `map`. `/nav/map`: `OccupancyGrid` en `map`.

## Nodos de Estructura

| Nodo | Qué hace | Interfaces |
|---|---|---|
| `base_driver_node` | ros2_control: `/cmd_vel` → velocidad de cada rueda; lee encoders | escucha `/cmd_vel`, `/base/emergency_stop`; publica `/base/wheel_odom` |
| `imu_driver_node` | Lee la IMU | publica `/base/imu` (frame `imu_link_1`, con covarianzas) |
| `odom_ekf_node` | `robot_localization` con un yaml | publica `/base/odom` y TF `odom -> base_footprint` |
| `start_sound_node` | Botón de inicio | `/base/start_sound`, `/base/start_button` (`std_msgs/Bool`) |
| `safety_monitor_node` | Botón de emergencia (rojo, el único rojo) | `/base/emergency_stop` (`std_msgs/Bool`) |
| `led_node` | Luces | servicio `/base/set_led_state` (`idle`, `listening`, `ready_handover`, `alert`) |
| `robot_state_publisher` | TF fijas del URDF y del brazo | `/tf_static`, `/tf` (lee `/arm/joint_states`) |

Cinemática de MARTHA si las ruedas son mecanum (`lx`, `ly` = mitad del largo y del ancho entre
ruedas; DI = delantera izquierda, TD = trasera derecha):

```
wDI = (vx - vy - (lx+ly)w)/r    wDD = (vx + vy + (lx+ly)w)/r
wTI = (vx + vy - (lx+ly)w)/r    wTD = (vx - vy + (lx+ly)w)/r
```

Los signos dependen del montaje. Calibración: radio, `lx`, `ly` y signo de cada rueda; probar
avanzar, ir de lado y girar.

Prueba de Estructura sin Navegación: `ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist
"{linear: {x: 0.2}}"` durante 5 s → avanza 1 m y `/base/odom` marca cerca de 1 m. Con
`angular.z: 0.5` durante 12,6 s da una vuelta completa.

## MILO y MARTHA

Mismo contrato, mismos nodos; cambian el URDF, los drivers y la configuración. Un argumento
`robot:=milo|martha` en el launch elige URDF, yaml del controlador, configuración de Nav2 y zona base.
Los rosbag y videos llevan el nombre del robot.

| | MILO | MARTHA |
|---|---|---|
| Base | Hoverboard, 2 ruedas, diferencial | 4 ruedas omni/mecanum, holonómica |
| Usa de `/cmd_vel` | `linear.x`, `angular.z` | `linear.x`, `linear.y`, `angular.z` |
| Controlador | `diff_drive_controller` | Omnidireccional, propio si `ros2_controllers` no trae uno |
| Nav2 | Modelo DiffDrive | Modelo Omni; DWB o MPPI (Regulated Pure Pursuit no sirve de lado) |
| Filtro de seguridad | Adelante y atrás | También a los lados |
| Mirar a una persona | Gira la base, quieto | Puede girar mientras va de lado |
| Cámaras | Cabeza RGB, garra RGB-D | Cabeza RGB-D, garra RGB-D |
| LiDAR | RPLIDAR C1 | En la página "por confirmar"; en este workspace es el **RPLIDAR A2M8** |
| `ROS_DOMAIN_ID` (ejemplo de la página) | 11 | 12 |

Cada robot usa un `ROS_DOMAIN_ID` distinto para no obedecer el `/cmd_vel` del otro, y cada persona
que simula en la misma red usa el suyo.

## Simulación y robot real

Lo único que cambia es la capa de abajo: Gazebo reemplaza drivers y hardware y publica **los mismos
tópicos**. Lo que cambia es quién publica, no qué se publica.

| Pieza | Equipo | Gazebo | Robot real | Tópico |
|---|---|---|---|---|
| LiDAR | Navegación | `libgazebo_ros_ray_sensor` | `sllidar_node` | `/nav/scan` |
| Ruedas y odometría | Estructura | MARTHA: plugin omni (`libgazebo_ros_planar_move`) | ros2_control, 4 motores | `/cmd_vel` → `/base/wheel_odom` |
| IMU | Estructura | `libgazebo_ros_imu_sensor` | driver de la IMU | `/base/imu` |
| Fusión | Estructura | `odom_ekf_node`, mismo yaml | `odom_ekf_node`, mismo yaml | `/base/odom` |
| Cámaras | Visión | plugins (faltan en el URDF) | cámaras reales | `/vision/camera/*`, `/vision/gripper_camera/*` |
| Brazo | Brazo y Garra | stub con tiempos y `fail_rate` | motores | `/arm/*` |
| Botones | Estructura | `ros2 topic pub` a mano o botón en RViz | botones físicos | `/base/start_*`, `/base/emergency_stop` |
| Luces | Estructura | marcador de color en RViz | LEDs | `/base/set_led_state` |

- `use_sim_time: true` en todos los nodos en simulación, `false` en el robot. Uno solo mal y las TF
  "llegan del futuro": Nav2 y SLAM fallan sin un error claro.
- Launches: `sim.launch.py` y `robot.launch.py` lanzan lo mismo arriba; cambia solo la capa de abajo.

Reglas para que el paso a real no duela:

1. Mismos nombres, tipos y frames en simulación y en el robot. Nadie cambia su código.
2. Simular las limitaciones reales: LiDAR con ruido y la estructura que lo tapa, límites reales de
   velocidad y aceleración de las ruedas.
3. Stubs que tardan y fallan (parámetros de tiempo y de falla).
4. Grabar el robot real cuanto antes: `ros2 bag record /nav/scan /base/odom /base/imu` + cámaras.
5. Un `ROS_DOMAIN_ID` por persona.
6. Probar el reto completo en simulación (Gazebo + stubs + HRI en un computador).

Primera prueba de cualquier driver nuevo: `ros2 topic list` y `ros2 topic echo` con los nombres del
contrato.

## Diferencias conocidas entre la página y este workspace

Estado al 2026-10-24. La página dice cómo debe quedar el robot; aquí, lo que todavía no coincide.

**Robot y simulación**

- **Frame de la IMU**: el contrato dice `imu_link_1` (el nombre del URDF de MILO); el URDF de
  MARTHA usa `imu_link`. Hay que decidir uno antes de unir los dos robots.
- **LiDAR de MARTHA**: la página lo tiene "por confirmar"; es el RPLIDAR A2M8 (`sllidar_ros2`),
  0.19 m delante de `base_link`, con el plano del scan a 0.231 m del suelo (URDF medido el
  2026-10-21).
- **`/base/odom` en Gazebo**: `planar_move` lo publica directo, sin `/base/wheel_odom` ni EKF. Está
  permitido por la página mientras el EKF no exista.
- **Base real**: todavía no hay `base_driver_node`, `imu_driver_node` ni EKF en este repo. Lo
  decidido es la ESP32 con micro-ROS (publica `/base/imu` y el estado de cada rueda) y en el
  laptop ros2_control con `topic_based_ros2_control` + `mecanum_drive_controller`, que publica
  `/base/wheel_odom`.

**Navegación (`andesrobot_navigation`, Nav2)**

- `path_planner_node` de la página es Nav2 completo (planner, controller DWB, behaviors,
  bt_navigator, velocity_smoother). Atiende `/nav/navigate_to_pose` y publica
  `/nav/cmd_vel_planner`. La localización es AMCL con el modelo Omni sobre `/nav/map`.
- **Sin filtro de seguridad todavía**: el mux (`twist_mux`) publica directo en `/cmd_vel`. Falta
  `safety_filter_node` (que en MARTHA tiene que mirar también a los lados) y `/nav/safety_state`.
- **Teleop en el mux**: `/cmd_vel_teleop` entra al mux con la prioridad más alta (100, sobre mirar
  20 y navegar 10). Sirve para probar y mapear; en el launch de competencia no se lanza.
- **Filtro de la torre**: `filtro_torre.yaml` (laser_filters) borra los postes de la torre del scan
  y publica `/nav/scan_filtered`. Es interno de Navegación, como dice la página.
- **Falta todavía**: `/nav/pose`, el servicio `/nav/set_initial_pose`, el modo mapeo (`slam_node`)
  y los nodos `person_follower_node`, `look_controller_node` y `door_detector_node` con sus
  interfaces (`/nav/follow_person`, `/nav/look`, `/nav/look_ahead`, `/nav/detect_door`).

## Contrato completo

Estados: **contrato** (de la propuesta), **trazabilidad** (de la trazabilidad, tipo propuesto en la
página), **nuevo** (propuesto para cerrar un hueco), **decidido** (por el equipo), **opcional**.
**interna** = no cruza equipos.

| Interfaz | Clase | Tipo | Publica u ofrece | La usa | Estado |
|---|---|---|---|---|---|
| `/cmd_vel` | topic | geometry_msgs/Twist | safety_filter_node | base_driver_node | contrato |
| `/nav/cmd_vel_mux` | topic | geometry_msgs/Twist | cmd_vel_mux | safety_filter_node | decidido · interna |
| `/nav/safety_state` | topic | std_msgs/String | safety_filter_node | task_orchestrator_node | decidido |
| `/nav/cmd_vel_planner` | topic | geometry_msgs/Twist | path_planner_node | cmd_vel_mux | decidido |
| `/nav/cmd_vel_look` | topic | geometry_msgs/Twist | look_controller_node | cmd_vel_mux | decidido |
| `/base/odom` | topic | nav_msgs/Odometry | odom_ekf_node | localization_node, slam_node, path_planner_node | contrato |
| `/nav/scan` | topic | sensor_msgs/LaserScan | nav_drivers | slam_node, localization_node, path_planner_node, door_detector_node, safety_filter_node, person_detector_node | decidido |
| `/base/imu` | topic | sensor_msgs/Imu | imu_driver_node | odom_ekf_node | decidido |
| `/base/wheel_odom` | topic | nav_msgs/Odometry | base_driver_node | odom_ekf_node | decidido · interna |
| `/vision/camera/color` | topic | sensor_msgs/Image | camera_driver_node | person_detector_node, face_recognition_node, gesture_node, object_detector_node | decidido |
| `/vision/camera/depth` | topic | sensor_msgs/Image | camera_driver_node | person_detector_node, object_detector_node | decidido |
| `/hri/audio` | topic | audio_common_msgs/AudioData | audio_capture_node | asr_node, doorbell_detector_node | decidido · interna |
| `/hri/doorbell_detected` | topic | std_msgs/Bool | doorbell_detector_node | task_orchestrator_node | decidido |
| `/base/start_sound` | topic | std_msgs/Bool | start_sound_node | task_orchestrator_node | contrato |
| `/base/start_button` | topic | std_msgs/Bool | start_sound_node | task_orchestrator_node | decidido |
| `/base/emergency_stop` | topic | std_msgs/Bool | safety_monitor_node | base_driver_node, arm_controller_node, task_orchestrator_node | contrato |
| `/base/set_led_state` | service | SetLedState.srv | led_node | task_orchestrator_node, arm_controller_node | contrato |
| `/hri/play_sound` | service | PlaySound.srv | speaker_node | task_orchestrator_node, tts_node | decidido · interna |
| `/tf, /tf_static` | tf | tf2_msgs/TFMessage | robot_state_publisher |  | contrato |
| `/nav/map` | topic | nav_msgs/OccupancyGrid | competencia:localization_node, mapeo:slam_node | path_planner_node | contrato |
| `/nav/pose` | topic | geometry_msgs/PoseStamped | competencia:localization_node, mapeo:slam_node | task_orchestrator_node, person_follower_node, look_controller_node | contrato |
| `/nav/set_initial_pose` | service | SetInitialPose.srv | localization_node | task_orchestrator_node | nuevo |
| `/nav/navigate_to_pose` | action | NavigateToPose.action (Nav2) | path_planner_node | task_orchestrator_node | contrato |
| `/nav/follow_person` | action | FollowPerson.action | person_follower_node | task_orchestrator_node | contrato |
| `/nav/look` | action | LookAt.action | look_controller_node | task_orchestrator_node | trazabilidad |
| `/nav/look_ahead` | service | std_srvs/SetBool | look_controller_node | task_orchestrator_node | trazabilidad |
| `/nav/detect_door` | service | DetectDoor.srv | door_detector_node | task_orchestrator_node | decidido |
| `/vision/detect_handle` | service | DetectHandle.srv | object_detector_node | task_orchestrator_node | decidido |
| `/vision/gripper_camera/color` | topic | sensor_msgs/Image | camera_driver_node | object_detector_node | nuevo · interna |
| `/vision/gripper_camera/depth` | topic | sensor_msgs/Image | camera_driver_node | object_detector_node | nuevo · interna |
| `/vision/gripper_objects` | topic | ObjectArray.msg | object_detector_node | arm_controller_node, handover_detector_node | nuevo |
| `/vision/persons` | topic | PersonArray.msg | person_detector_node | task_orchestrator_node, person_follower_node, look_controller_node | contrato |
| `/vision/faces` | topic | FaceArray.msg | face_recognition_node | task_orchestrator_node | contrato |
| `/vision/gestures` | topic | GestureEvent.msg | gesture_node | task_orchestrator_node | contrato |
| `/vision/gaze` | topic | GazeTarget.msg | person_detector_node | look_controller_node | trazabilidad |
| `/vision/objects` | topic | ObjectArray.msg | object_detector_node | path_planner_node, task_orchestrator_node | opcional |
| `/vision/identify_person` | service | IdentifyPerson.srv | face_recognition_node | task_orchestrator_node | contrato |
| `/vision/describe_person` | service | DescribePerson.srv | face_recognition_node | task_orchestrator_node | contrato |
| `/vision/detect_bag` | service | DetectBag.srv | object_detector_node | task_orchestrator_node | contrato |
| `/vision/detect_free_seat` | service | DetectFreeSeat.srv | object_detector_node | task_orchestrator_node | nuevo |
| `/hri/speech_text` | topic | std_msgs/String | asr_node | dialogue_manager_node | contrato |
| `/hri/say` | topic | std_msgs/String | dialogue_manager_node | tts_node | contrato |
| `/hri/dialogue_state` | topic | DialogueState.msg | dialogue_manager_node | task_orchestrator_node | contrato |
| `/hri/remember` | service | RememberGuest.srv | dialogue_manager_node | task_orchestrator_node | trazabilidad · interna |
| `/hri/converse` | action | Converse.action | dialogue_manager_node | task_orchestrator_node | nuevo · interna |
| `/hri/mode` | topic | std_msgs/String | task_orchestrator_node | dialogue_manager_node, llm_agent_node | nuevo · interna |
| `/hri/world_context` | topic | WorldContext.msg | task_orchestrator_node | llm_agent_node | nuevo · interna |
| `/hri/llm_chat` | action | LlmChat.action | llm_agent_node | dialogue_manager_node | nuevo · interna |
| `/hri/skill_request` | service | RequestSkill.srv | task_orchestrator_node | llm_agent_node | nuevo · interna |
| `/arm/joint_states` | topic | sensor_msgs/JointState | arm_controller_node | robot_state_publisher, grasp_planner_node | contrato |
| `/arm/gripper_state` | topic | GripperState.msg | gripper_controller_node | handover_detector_node, task_orchestrator_node | contrato |
| `/arm/pick_object` | action | PickObject.action | arm_controller_node | task_orchestrator_node | contrato |
| `/arm/place_object` | action | PlaceObject.action | arm_controller_node | task_orchestrator_node | contrato |
| `/arm/point_to` | action | PointTo.action | arm_controller_node | task_orchestrator_node | nuevo |
| `/arm/move_to_pose` | action | MoveArmToPose.action | arm_controller_node | task_orchestrator_node | nuevo |
| `/arm/open_door` | action | OpenDoor.action | arm_controller_node | task_orchestrator_node | decidido |
| `/arm/gesture` | action | Gesture.action | arm_controller_node | task_orchestrator_node | nuevo |
| `/arm/detect_handover` | service | DetectHandover.srv | handover_detector_node | arm_controller_node | contrato · interna |

Nombres viejos que **no** se usan (se dejaron anotados en la página para que nadie se suscriba a ellos):

- `/nav/imu`: Movido: la IMU es de Estructura (/base/imu), porque de ella sale la odometría.
- `/base/scan`: Movido: ahora es /nav/scan, publicado por nav_drivers de Navegación.
- `/base/camera/color`: Movido: ahora es /vision/camera/color, de Visión.
- `/base/camera/depth`: Movido: ahora es /vision/camera/depth, de Visión.
- `/base/audio`: Movido: ahora es /hri/audio, publicado por audio_capture_node de Interacción.
- `/base/doorbell_detected`: Movido: ahora es /hri/doorbell_detected, publicado por doorbell_detector_node (Interacción Humano-Robot).
- `/base/play_sound`: Movido: ahora es /hri/play_sound, atendido por speaker_node de Interacción.
- `/base/head_cmd`: No aplica: el equipo decidió no usar cabeza pan-tilt.
