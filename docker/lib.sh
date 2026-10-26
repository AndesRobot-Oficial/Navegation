#!/bin/bash
# Funciones comunes de sim.sh y robot.sh (no se ejecuta solo).
# Uso: source docker/lib.sh; martha_init <sim|robot> [cpu]
#
# "source" significa que sim.sh / robot.sh cargan estas funciones dentro de su propio proceso,
# como si las hubieran escrito ellos. Así no repetimos el mismo código en los dos scripts.

# ---------------------------------------------------------------------------------------------
# martha_init: prepara las variables que necesita "docker compose".
#   $1 = modo: "sim" (tu PC) o "robot" (portátil de Martha, con acceso a los USB)
#   $2 = opción extra: "cpu" (no usar NVIDIA) o "software" (dibujar con la CPU)
# Al final deja lista la variable DC = el comando docker compose con los archivos correctos.
# ---------------------------------------------------------------------------------------------
martha_init() {
  # "local" = variables que solo existen dentro de esta función.
  local mode="$1" force_cpu="$2"
  # UID/GID = tu número de usuario y de grupo en el PC. compose.yaml se los pasa al Dockerfile
  # para crear dentro del contenedor un usuario igual a ti: así lo que se compila en ros2_ws
  # queda a tu nombre y no de "root". (UID ya existe en bash; "export" lo hace visible a docker.)
  export UID GID="$(id -g)"
  # GIDs del host para que el contenedor pueda usar la GPU (/dev/dri)
  # getent group video -> "video:x:44:..." ; cut -d: -f3 se queda con el 3er campo = el número 44.
  export MARTHA_VIDEO_GID="$(getent group video | cut -d: -f3)"
  export MARTHA_RENDER_GID="$(getent group render | cut -d: -f3)"
  # ":" es un comando que no hace nada; se usa para el truco ${VAR:=valor}, que asigna un valor
  # por defecto si la variable quedó vacía (por ejemplo, un PC sin grupo "render").
  : "${MARTHA_VIDEO_GID:=44}"
  # Docker no acepta el mismo grupo dos veces en group_add: si no hay grupo "render" o tiene el
  # mismo número que "video", usamos el de "dialout" (puertos serie), que es inofensivo.
  if [ -z "$MARTHA_RENDER_GID" ] || [ "$MARTHA_RENDER_GID" = "$MARTHA_VIDEO_GID" ]; then
    MARTHA_RENDER_GID="$(getent group dialout | cut -d: -f3)"; MARTHA_RENDER_GID="${MARTHA_RENDER_GID:-20}"
  fi
  # Modo "software": compose.yaml pasa esta variable como LIBGL_ALWAYS_SOFTWARE=1 (OpenGL por CPU).
  [ "$force_cpu" = "software" ] && export MARTHA_SOFTWARE_GL=1
  # Archivo base de compose, siempre.
  MARTHA_FILES="-f docker/compose.yaml"
  # En el robot se suma compose.robot.yaml (acceso a /dev, o sea a los USB del lidar y del ESP32).
  [ "$mode" = "robot" ] && MARTHA_FILES="$MARTHA_FILES -f docker/compose.robot.yaml"
  # Si el PC tiene NVIDIA (existe nvidia-smi) y Docker tiene el runtime de NVIDIA instalado,
  # se suma compose.nvidia.yaml para que el contenedor use esa GPU. Con "cpu" o "software" no.
  if [ "$force_cpu" != "cpu" ] && [ "$force_cpu" != "software" ] && command -v nvidia-smi >/dev/null && docker info 2>/dev/null | grep -qi nvidia; then
    MARTHA_FILES="$MARTHA_FILES -f docker/compose.nvidia.yaml"
  fi
  # Comando final que usan todas las demás funciones, por ejemplo: $DC up -d
  DC="docker compose $MARTHA_FILES"
}

# Permite que el contenedor abra ventanas (RViz, Gazebo). Sin pantalla, no hace nada.
martha_display() {
  # -n = "no está vacío". Si DISPLAY está vacío (computador sin pantalla), salimos sin error.
  [ -n "$DISPLAY" ] || return 0
  if command -v xhost >/dev/null; then
    # xhost controla quién puede dibujar ventanas en tu pantalla. Damos permiso a tu usuario
    # y a los procesos locales (el contenedor corre en este mismo PC).
    xhost +si:localuser:"$(whoami)" >/dev/null
    # ">/dev/null 2>&1" esconde la salida; "|| true" evita que un error aquí detenga el script.
    xhost +local:docker >/dev/null 2>&1 || true
  else
    echo "[martha] Falta xhost: sudo apt install x11-xserver-utils"
  fi
}

# Muestra con qué está dibujando OpenGL dentro del contenedor.
# Si dice tu tarjeta (Intel/AMD/NVIDIA) hay aceleración; si dice "llvmpipe" está usando la CPU.
martha_gpu_info() {
  martha_run 'glxinfo -B 2>/dev/null | grep -E "OpenGL (vendor|renderer)|Accelerated" || echo "glxinfo falló: no hay contexto OpenGL"'
}

# Levanta el contenedor (construye la imagen si cambió) y compila la 1a vez.
martha_up() {
  martha_display
  # up = crear/arrancar el contenedor; -d = en segundo plano; --build = reconstruir la imagen
  # si el Dockerfile cambió (si no cambió, Docker reutiliza lo ya construido y es instantáneo).
  $DC up -d --build
  # Dentro del contenedor ("exec ros"): si todavía no existe install/setup.bash (nunca se compiló),
  # compila el workspace. "bash -ic" = bash interactivo, para que cargue ~/.bashrc con ROS.
  # También compila si apareció un paquete nuevo en src/ que todavía no está en install/
  # (por ejemplo andesrobot_navigation después de un git pull).
  $DC exec ros bash -ic 'cd /ros2_ws && falta=0; [ -f install/setup.bash ] || falta=1; for p in $(colcon list -n 2>/dev/null); do [ -d install/$p ] || falta=1; done; [ $falta = 0 ] || colcon build --symlink-install'
}

# Corre un comando dentro del contenedor con ROS cargado.
# "$*" = todos los argumentos de la función juntos como un solo texto.
martha_run() {
  $DC exec ros bash -ic "$*"
}

# ¿Está corriendo el contenedor? "ps -q ros" devuelve su ID si existe; vacío si no.
martha_running() {
  [ -n "$($DC ps -q ros 2>/dev/null)" ]
}

# Igual que martha_running, pero si NO está corriendo avisa y termina el script.
# $0 = nombre del script que llamó (./sim.sh o ./robot.sh), para decirte qué correr primero.
martha_need_running() {
  martha_running || { echo "[martha] El contenedor no está levantado. Primero: $0"; exit 1; }
}

# Guarda el mapa que está armando slam_toolbox (/nav/map) como .pgm (imagen) + .yaml (metadatos).
# Va a /maps del contenedor = la carpeta maps/ del repo de Navegación en tu PC: no se pierde al
# apagar el contenedor, y ./sim.sh nav <nombre> / ./robot.sh nav <nombre> lo usan tal cual.
# (Solo el repo de Navegación monta /maps: en el de Estructura no se mapea.)
martha_save_map() {
  local name="$1"
  martha_need_running
  # No pisar un mapa que ya existe (casa, lab_robotica...): elige otro nombre o bórralo a mano.
  [ ! -e "maps/$name.yaml" ] || { echo "[martha] Ya existe maps/$name.yaml: usa otro nombre"; exit 1; }
  # map_saver_cli espera el mapa en el tópico -t y lo escribe en -f (sin extensión).
  martha_run "ros2 run nav2_map_server map_saver_cli -f /maps/$name -t /nav/map"
  echo "[martha] Mapa guardado en maps/$name.pgm / .yaml. Sus lugares van en maps/${name}_lugares.yaml"
}

# Teleop por teclado. Sin navegación publica directo en /cmd_vel, que es lo que mueve a Martha.
# Con ./sim.sh nav corriendo (existe el nodo twist_mux) publica en /cmd_vel_teleop, la entrada de
# mayor prioridad del mux: mientras tocas teclas le quitas el control a Nav2, y medio segundo
# después de soltar, Nav2 lo recupera (ver andesrobot_navigation/config/twist_mux.yaml).
# Martha es holonómica: con Shift apretado (U I O J K L M < >) el teclado manda también linear.y
# y Martha se mueve de lado.
TELEOP_CMD='if ros2 node list 2>/dev/null | grep -qx /twist_mux; then echo "[martha] Navegación corriendo: el teclado entra al mux por /cmd_vel_teleop"; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop; else ros2 run teleop_twist_keyboard teleop_twist_keyboard; fi'
