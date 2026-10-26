#!/bin/bash
# Martha FÍSICA — en el laptop de Martha (RPLIDAR A2M8 por USB). La parte de Navegación.
#
#   ./robot.sh               levanta el contenedor con acceso a los USB
#   ./robot.sh lidar         LiDAR + modelo + RViz: ver /nav/scan (no necesita la base)
#   ./robot.sh mapear        mapeo: LiDAR + slam_toolbox + RViz (manejar con ./robot.sh teleop)
#   ./robot.sh mapa [nombre] (otra terminal) guardar el mapa en maps/  (default: martha_AAAAMMDD_HHMM)
#   ./robot.sh nav <mapa>    navegación autónoma (AMCL + Nav2) con un mapa de maps/
#   ./robot.sh ir <lugar>    (otra terminal) mandar a Martha a un lugar del mapa  (--lista)
#   ./robot.sh teleop        (otra terminal) manejar a Martha con el teclado
#   ./robot.sh rviz          abrir RViz aparte (en este laptop o en otro PC de la red)
#   ./robot.sh puertos       lista los puertos USB conectados (para docker/udev)
#   ./robot.sh shell | build | stop
#
# mapear, nav y teleop necesitan la BASE corriendo (repo de Estructura): /base/odom, la TF
# odom -> base_footprint y alguien que escuche /cmd_vel. Puerto del LiDAR: por defecto
# /dev/martha_lidar (ver docker/udev/README.md). Sin udev:  LIDAR=/dev/ttyUSB0 ./robot.sh mapear
set -e
# (Las líneas 2-17 de arriba son la ayuda que imprime "sed -n '2,17p'" si escribes una opción
#  que no existe. Los comentarios explicativos van desde aquí.)
#
# Es el mismo esquema del robot.sh de milo_ws y del de Estructura: este script corre en el HOST
# (el laptop de Martha) y todo lo de ROS corre dentro del contenedor "martha" en modo robot
# (docker/compose.robot.yaml = acceso a /dev, o sea a los USB). El contenedor de Estructura
# (martha_estructura) corre al lado, en el mismo laptop, con la base; los dos usan la red del host
# y el mismo ROS_DOMAIN_ID, así que se ven los tópicos.
#
# "set -e": si un comando falla, el script se detiene.
# Ir a la carpeta de este script y cargar las funciones comunes (martha_init, martha_up...).
cd "$(dirname "$0")"
source docker/lib.sh

# Puerto del LiDAR. ${LIDAR:-/dev/martha_lidar} = usa la variable LIDAR si la escribiste antes del
# comando (LIDAR=/dev/ttyUSB0 ./robot.sh mapear); si no, el nombre fijo que crean las reglas udev.
LIDAR="${LIDAR:-/dev/martha_lidar}"
# RViz junto con Martha: sí si hay pantalla (el laptop de Martha), no si se corre sin pantalla
# (por SSH). Se puede forzar: RVIZ=false ./robot.sh mapear
if [ -n "$DISPLAY" ]; then RVIZ="${RVIZ:-true}"; else RVIZ="${RVIZ:-false}"; fi
# robot_state_publisher con el URDF de Martha. Es de Estructura, pero su bringup todavía no
# existe: lo lanzamos aquí. Cuando exista: MODELO=false ./robot.sh ...
MODELO="${MODELO:-true}"

# Primer "case": opciones que NO necesitan el contenedor en modo robot.
case "$1" in
  puertos)
    # Recorre todos los puertos serie USB conectados (ttyUSB = adaptadores tipo CP2102, como el
    # del ESP32 y el del RPLIDAR; ttyACM = placas con USB nativo) y muestra sus datos de fábrica
    # para las reglas udev.
    for d in /dev/ttyUSB* /dev/ttyACM*; do
      # Si no hay ninguno, el patrón queda literal ("/dev/ttyUSB*"): "-e" lo descarta.
      [ -e "$d" ] || continue
      echo "== $d"
      # udevadm muestra los atributos del dispositivo; grep -m3 se queda con los 3 primeros
      # idVendor / idProduct / serial (los que se copian en docker/udev/99-martha.rules).
      udevadm info -a -n "$d" | grep -m3 -E 'ATTRS\{(idVendor|idProduct|serial)\}'
    done
    # Si las reglas udev ya están instaladas, se ven los enlaces fijos.
    ls -l /dev/martha_esp32 /dev/martha_lidar 2>/dev/null || true
    exit 0 ;;
  rviz)
    # Si el contenedor ya está corriendo (Martha andando en este mismo laptop): abrir RViz DENTRO
    # de ese contenedor, sin reiniciarlo. Si no (otro PC de la red): contenedor normal (sin USB),
    # solo RViz, que ve los tópicos de Martha por la red (misma red y mismo ROS_DOMAIN_ID).
    martha_init robot
    if ! martha_running; then martha_init sim; martha_up; fi
    martha_display
    # navigation.rviz sirve para todo: mapa, lidar, Martha, y además Nav2 si está corriendo.
    exec $DC exec ros bash -ic 'rviz2 -d $(ros2 pkg prefix andesrobot_navigation)/share/andesrobot_navigation/rviz/navigation.rviz' ;;
esac

# Desde aquí: modo robot (docker compose suma compose.robot.yaml = acceso a los USB).
martha_init robot
case "$1" in
  # Mismas opciones que en sim.sh (ver comentarios allá y en docker/lib.sh).
  teleop) martha_need_running; exec $DC exec ros bash -ic "$TELEOP_CMD" ;;
  mapa)   martha_save_map "${2:-martha_$(date +%Y%m%d_%H%M)}"; exit 0 ;;
  shell)  martha_need_running; exec $DC exec ros bash ;;
  build)  martha_need_running; exec $DC exec ros bash -ic 'cd /ros2_ws && colcon build --symlink-install' ;;
  stop)   $DC down; exit 0 ;;
  # Ir a lugares con nombre. Usa los lugares del mapa con que está corriendo ./robot.sh nav: se
  # los pregunta al nodo de los marcadores (su parámetro "ubicaciones" = /maps/<mapa>_lugares.yaml).
  ir)     martha_need_running; shift
          exec $DC exec ros bash -ic "U=\$(ros2 param get /ubicaciones_rviz ubicaciones 2>/dev/null | sed -n 's/^String value is: //p'); [ -n \"\$U\" ] || { echo '[martha] No hay navegación corriendo: primero ./robot.sh nav <mapa>'; exit 1; }; ros2 run andesrobot_navigation ir_a --ubicaciones \"\$U\" $*" ;;
  # Navegación: necesita un mapa en maps/ (<mapa>.yaml + .pgm). Sus lugares (<mapa>_lugares.yaml)
  # son opcionales: sin ese archivo navega igual, pero solo con "2D Goal Pose" en RViz.
  nav)    [ -n "$2" ] && [ -f "maps/$2.yaml" ] || { echo "[martha] Uso: ./robot.sh nav <mapa>. Mapas en maps/:"; ls maps/*.yaml 2>/dev/null | grep -v _lugares | xargs -n1 basename | sed 's/.yaml$//; s/^/  /'; exit 1; } ;;
  # Estas siguen abajo: revisar el LiDAR y levantar el contenedor.
  ""|lidar|mapear) ;;
  # Opción desconocida: mostrar ayuda.
  *) sed -n '2,17p' "$0"; exit 1 ;;
esac

# Levantar el contenedor (con USB) y compilar si es la primera vez.
martha_up
# Sin palabra: solo dejar el contenedor listo.
if [ -z "$1" ]; then
  echo
  echo "[martha] Contenedor de Navegación listo, con acceso a los USB. Puertos fijos:"
  ls -l /dev/martha_esp32 /dev/martha_lidar 2>/dev/null || echo "[martha]   (sin reglas udev todavía: ./robot.sh puertos y docker/udev/README.md)"
  echo "[martha] Probar el LiDAR:  ./robot.sh lidar      Mapear:  ./robot.sh mapear"
  exit 0
fi

# Desde aquí todo usa el LiDAR: si no está conectado, mejor no arrancar a medias.
[ -e "$LIDAR" ] || { echo "[martha] No existe $LIDAR. Revisa la conexión, ./robot.sh puertos, o usa LIDAR=/dev/ttyUSB0 ./robot.sh $*"; exit 1; }

# LiDAR solo: el driver + el modelo + el filtro de la torre + RViz (robot_lidar.launch.py). No
# necesita la base: sirve para revisar que el scan llega (~10 Hz) y que el filtro borra los postes.
if [ "$1" = "lidar" ]; then
  echo
  echo "[martha] LiDAR en $LIDAR. Otra terminal:  ./robot.sh shell  y  ros2 topic hz /nav/scan  (~10 Hz)"
  echo
  exec $DC exec ros bash -ic "ros2 launch andesrobot_navigation robot_lidar.launch.py lidar_port:=$LIDAR modelo:=$MODELO rviz:=$RVIZ"
fi

# mapear y nav necesitan la odometría de la base. Esperar hasta 5 s un mensaje de /base/odom; si no
# llega, avisar (pero seguir: puede que la base arranque después). --no-daemon = preguntar directo,
# sin el "ros2 daemon" (un proceso que guarda la lista de tópicos y a veces queda pegado).
if ! martha_run 'timeout 5 ros2 topic echo --once --no-daemon /base/odom nav_msgs/msg/Odometry >/dev/null 2>&1'; then
  echo "[martha] OJO: no llega /base/odom. Sin la base (repo de Estructura) Martha no se mueve y"
  echo "[martha]      slam_toolbox / AMCL no tienen la TF odom -> base_footprint."
fi

if [ "$1" = "mapear" ]; then
  echo
  echo "[martha] Mapeo (LiDAR $LIDAR, RViz: $RVIZ). El mapa nace donde está Martha ahora. Otra terminal:"
  echo "[martha]   ./robot.sh teleop            manejar a Martha (despacio, volviendo por lugares ya vistos)"
  echo "[martha]   ./robot.sh mapa <nombre>     guardar el mapa en maps/<nombre>.pgm / .yaml"
  echo
  exec $DC exec ros bash -ic "ros2 launch andesrobot_slam robot_mapping.launch.py lidar_port:=$LIDAR modelo:=$MODELO rviz:=$RVIZ"
fi

# nav: AMCL parte en el origen del mapa = donde empezó el mapeo. Si Martha está en otro lado,
# "2D Pose Estimate" en RViz.
echo
echo "[martha] Navegación con el mapa maps/$2.yaml. Martha tiene que partir donde empezó ese mapeo"
echo "[martha] (si no: '2D Pose Estimate' en RViz). En otra terminal:"
echo "[martha]   ./robot.sh ir --lista        ver los lugares     ./robot.sh ir <lugar>   ir"
echo "[martha]   ./robot.sh teleop            el teclado le quita el control a Nav2 mientras lo usas"
echo
exec $DC exec ros bash -ic "ros2 launch andesrobot_navigation robot_navigation.launch.py map:=/maps/$2.yaml ubicaciones:=/maps/${2}_lugares.yaml lidar_port:=$LIDAR modelo:=$MODELO rviz:=$RVIZ"
