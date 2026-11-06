*martha_ws · ROS 2 Humble · Gazebo classic 11 · Nav2 · Docker*

# Guía: navegación con PPO, filtro de seguridad y búsqueda de objetos

Cómo probar en simulación lo que agregó el equipo de Navegación (Santiago) en noviembre de 2026.
Todo corre en Docker desde la raíz del repo; no hace falta instalar ROS en tu PC.

| Qué | Dónde | Estado |
|---|---|---|
| Navegación con el **PPO** de la tesis de Nicolás como controlador local | `andesrobot_navigation/ppo_controller.py`, `ros2_ws/src/martha_nav` (copia) | probado en Gazebo |
| **Filtro de seguridad** a 360° (NAV-11) | `andesrobot_safety` | probado en Gazebo; **aún no conectado** a la navegación (NAV-10) |
| **Búsqueda de objetos** con Visión: sala por sala, girando | `andesrobot_navigation/buscar_objeto.py`, `girar_buscando.py` | prueba con una visión simulada |

Nada de esto se ha probado en la Martha real.

## 0. Una vez

```bash
git clone https://github.com/shdeza12/Navigation-MARTHA.git ~/martha_ws_oficial
cd ~/martha_ws_oficial
```

La primera vez `./sim.sh` construye la imagen de Docker y compila (varios minutos). Si ya tienes
otra simulación abierta de Martha, ciérrala antes: usan el mismo contenedor (`./sim.sh stop`).

## 1. Navegación a los waypoints (con el PPO)

| Terminal | Comando | Qué hace |
|:-:|---|---|
| 1 | `./sim.sh nav` | Gazebo + Martha + Nav2 con el PPO + RViz |
| 2 | `./sim.sh ir --lista` | ver los 28 waypoints de la casa |
| 2 | `./sim.sh ir indoor base sofa table` | mandar a Martha, uno después del otro |

Espera dos veces **«Managed nodes are active»** en la terminal 1 antes de usar la 2. En el log tiene
que salir `PPO listo: /policies/martha_holo.npz`.

- Nav2 calcula la ruta y la política PPO decide las velocidades. Al llegar a 0.3 m de la meta, un
  ajuste final deja a Martha en la pose y orientación del waypoint (0.12 m, 0.15 rad).
- Para comparar con el controlador clásico de Nav2: `./sim.sh nav casa dwb`.
- En RViz también sirve **2D Goal Pose** (clic en el mapa y arrastrar para la orientación).

Cómo encaja (detalles en [`policies/README.md`](../../policies/README.md)):

```
bt_navigator ── /follow_path ──► ppo_controller ── /nav/cmd_vel_planner ──► twist_mux ──► /cmd_vel
 (NavFn: la ruta)               (la política .npz,                          ▲
                                 solo numpy)               girar_buscando ── /nav/cmd_vel_look
```

## 2. Filtro de seguridad (`safety_filter_node`)

Frena si hay algo a menos de 30 cm **en la dirección en que se mueve Martha**, también de lado y en
diagonal, y siempre la deja alejarse. Publica `/nav/safety_state` (`ok`, `slow`, `stop`). Todavía no
está en la cadena de `./sim.sh nav`: conectarlo es la tarea NAV-10.

Prueba con tres cajas (adelante, de lado y en diagonal), cada comando en su terminal, dentro de
`./sim.sh shell`:

```bash
ros2 launch andesrobot_gazebo sim.launch.py gui:=false rviz:=false world:=/usr/share/gazebo-11/worlds/empty.world x:=0.0 y:=0.0 yaw:=0.0
ros2 launch andesrobot_safety safety.launch.py use_sim_time:=true
python3 /ros2_ws/src/andesrobot_safety/scripts/prueba_caja.py
```

Tiene que terminar con `Resultado: OK`.

## 3. Búsqueda de objetos con Visión (prueba)

El flujo acordado con Visión el 2026-11-05: el orquestador le dice a Navegación a qué sala ir y a
Visión qué objeto buscar.

1. Martha va a la sala donde es más probable el objeto (`maps/casa_objetos.yaml`).
2. Gira 360° despacio (0.3 rad/s) mientras Visión responde si lo vio (`true`/`false`).
3. Si lo vio, sigue girando hasta dejar de verlo; el ángulo del medio es la dirección del objeto, y
   va al mueble de esa sala en esa dirección. Si no lo vio, va a la sala siguiente.
4. Al terminar, lo encuentre o no, vuelve a casa (el waypoint `base`).

| Terminal | Comando | Qué hace |
|:-:|---|---|
| 1 | `./sim.sh nav` | la navegación (incluye el nodo que gira) |
| 2 | `./sim.sh buscar sprite coffee_table` | la Sprite en la mesa de centro: primero la busca en la cocina |
| 2 | `./sim.sh buscar sprite counter` | la Sprite en el mesón de la cocina |

`./sim.sh buscar` lanza una **visión simulada** (`vision_simulada.py`, solo para probar): ve el
objeto si queda dentro de un cono de 60°, a menos de 4 m y sin un muro del mapa en medio. Con el
`object_detector_node` real de Visión se corre solo el orquestador de prueba:

```bash
ros2 run andesrobot_navigation buscar_objeto sprite              # vuelve a base al terminar
ros2 run andesrobot_navigation buscar_objeto sprite --casa ''    # se queda donde terminó
```

| Interfaz | Tipo | Quién | Estado |
|---|---|---|---|
| `/nav/search_rotate` | acción `nav2_msgs/Spin` | Navegación (`girar_buscando`) | nueva; falta en el contrato |
| `/vision/object_detected` | `std_msgs/Bool` | Visión | nombre de prueba: confirmar con Visión |
| `/vision/target_object` | `std_msgs/String` (latched) | orquestador | nombre de prueba: confirmar con Visión |
| `/nav/navigate_to_pose` | acción `NavigateToPose` | Navegación | del contrato |

**Probado en Gazebo:** la Sprite en `coffee_table` (cocina sin verla, living, la ve y se acerca a
`coffee_table`). **Sin probar:** la vuelta a `base` y la Sprite en `counter`. Como Visión solo dice
`true`/`false` y no dónde está el objeto, el mueble se elige por dirección: con muebles juntos puede
equivocarse. Para el robot real conviene que Visión entregue al menos el ángulo o la pose.

## 4. Ver qué pasa por dentro

Dentro de `./sim.sh shell`:

| Para ver | Comando |
|---|---|
| quién se conecta con quién | `ros2 run rqt_graph rqt_graph` |
| los tópicos en vivo | `ros2 run rqt_topic rqt_topic` |
| las acciones | `ros2 action list` |
| dónde está Martha en el mapa | `ros2 run tf2_ros tf2_echo map base_footprint` |
| qué responde Visión | `ros2 topic echo /vision/object_detected` |

Una imagen guardada desde rqt queda dentro del contenedor (por ejemplo `/home/ros/`). Para sacarla:
`docker cp martha:/home/ros/rosgraph.png ~/Downloads/`. Si la guardas dentro de `/ros2_ws/`, aparece
sola en `ros2_ws/` de tu PC.

## 5. Pruebas automáticas y problemas comunes

```bash
./sim.sh pruebas
```

| Problema | Solución |
|---|---|
| `ir` o `buscar` se quedan en *Esperando…* | Nav2 no terminó de arrancar: Ctrl+C en la terminal 1 y otra vez `./sim.sh nav` |
| no sale `PPO listo` en el log | se lanzó con `dwb`, o falta `policies/martha_holo.npz` |
| cambié código y no se nota | Ctrl+C y otra vez `./sim.sh nav` (recompila antes de lanzar) |
| el contenedor es de otra carpeta | `./sim.sh stop` desde la carpeta vieja y lanzar desde esta |

## Pendientes

- NAV-10: conectar el filtro de seguridad a la cadena (`twist_mux` → `/nav/cmd_vel_mux` → filtro).
- Acordar con Visión los tópicos de la búsqueda y agregarlos al contrato junto con
  `/nav/search_rotate`; pedir el ángulo o la pose del objeto.
- Probar en la Martha real (necesita la base de Estructura) y el mapa y la `base_zone` propios.
