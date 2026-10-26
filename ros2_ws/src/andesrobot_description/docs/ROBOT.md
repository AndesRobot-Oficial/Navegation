# MARTHA — modelo

**Este paquete vive en el repo de Estructura** (`Structure-Embedded-Systems_MATHA`). El repo de
Navegación tiene una **copia** para su simulación: cuando cambies algo aquí, copia la carpeta
`andesrobot_description` completa a `Navegation/ros2_ws/src/` y en el commit de Navegación di de
qué commit de Estructura viene. No se edita la copia.

Convención REP-103/105: +X adelante, +Y izquierda, +Z arriba. Archivos: `urdf/martha.urdf.xacro`
(geometría), `urdf/martha.sim.xacro` (plugins de Gazebo), `config/martha_controllers.yaml`
(ros2_control de las ruedas en Gazebo, con `drive:=mecanum`).

## Frames

| Frame | Dónde |
|---|---|
| `base_footprint` | raíz del árbol, en el suelo bajo el centro del robot |
| `base_link` | en el mismo punto que `base_footprint` (unión directa: las ruedas dejan el chasis a pocos mm del suelo) |
| `laser` | RPLIDAR A2M8, 0.19 m delante de `base_link`; plano del scan a 0.231 m del suelo |
| `imu_link` | IMU; posición provisoria (centro del chasis, z = 0.1375 m). El contrato dice `imu_link_1`, el nombre de Milo: falta decidir uno |
| `*_wheel` | las 4 ruedas (FL, FR, RL, RR) en x = ±0.21 m, y = ±0.175 m, z = 0.08 m |

## Base

- Ruedas mecanum de radio 0.075 m y ancho 0.045 m, con 12 rodillos a 45° cada una
  (`drive:=mecanum`). `lx + ly` = 0.385 m, igual que en el firmware.
- Caja de contacto 0.58 × 0.41 m (la usan la navegación y el filtro de la torre).

## Martha tal como está construida (`tower:=true`, medida el 2026-10-21)

- Dos tablas de 5 mm: la de abajo con la forma del chasis (tope a 0.095 m) y la de arriba de
  0.57 × 0.405 m (tope a 0.20 m). El LiDAR va sobre la de arriba.
- Torre de perfiles de aluminio: 4 postes de 20 × 40 mm (el lado largo en x) y 0.91 m de alto, en
  x = −0.033 m (adelante) y −0.208 m (atrás), y = ±0.11 m, con un travesaño bajo y uno alto por
  par. Los postes cruzan el plano del LiDAR: se ven en el scan (Navegación los borra con su
  filtro de la torre). Las masas y la sección de los travesaños son estimadas.
- `tower:=false` es el modelo original de la tesis, sin tablas ni torre y con el LiDAR a 0.2325 m.

## Argumentos del xacro

| Arg | Default | Uso |
|---|---|---|
| `drive` | `planar` | `planar`: ruedas fijas y Gazebo mueve el cuerpo con `/cmd_vel`. `mecanum`: ruedas con rodillos y ros2_control |
| `tower` | `true` | la torre y las dos tablas (Martha real) |
| `sim` | `false` | `true` agrega los plugins de Gazebo (`martha.sim.xacro`) |
| `lidar_samples` | `360` | rayos por vuelta del LiDAR simulado |
| `controllers_file` | `config/martha_controllers.yaml` | controladores de ros2_control con `drive:=mecanum` |

**Nunca escribas dos puntos seguidos de un espacio** en los `.xacro`, ni en comentarios: con
`drive:=mecanum`, `gazebo_ros2_control` lee el URDF como YAML y falla sin avisar.
`test/test_urdf.py` lo revisa.
