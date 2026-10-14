#!/bin/bash
# Martha en SIMULACIÓN (Gazebo + RViz) — en tu PC.
#
#   ./sim.sh                 levanta todo: Gazebo con la casa RoboCup@Home, Martha y RViz
#   ./sim.sh teleop          (otra terminal) manejar a Martha con el teclado
#   ./sim.sh modelo          solo el modelo en RViz, sin Gazebo (revisar el URDF)
#   ./sim.sh pruebas         correr las pruebas (pytest) de los paquetes
#   ./sim.sh shell           terminal dentro del contenedor
#   ./sim.sh build           recompilar el workspace
#   ./sim.sh stop            apagar el contenedor
#   ./sim.sh mecanum         igual que ./sim.sh pero con las ruedas mecanum (ros2_control)
#   ./sim.sh cpu             igual que ./sim.sh pero sin GPU NVIDIA
#   ./sim.sh sin-gazebo      sin la ventana de Gazebo (más liviano; RViz igual se abre)
#   ./sim.sh software        render por CPU (si Gazebo/RViz se ven NEGROS); más lento
#   ./sim.sh gpu             muestra con qué tarjeta está dibujando el contenedor
set -e
# (Las líneas 2-15 de arriba son la ayuda: el "sed -n '2,15p'" de más abajo las imprime
#  cuando escribes una opción que no existe. Por eso los comentarios explicativos van aquí.)
#
# Es el mismo esquema de milo_ws: el código vive en tu PC (martha_ws/ros2_ws) y el contenedor lo
# ve como /ros2_ws. Editas en tu PC y compilas/corres dentro de Docker con este script.
#
# Ir a la carpeta donde está este script (martha_ws), aunque lo llames desde otra parte.
cd "$(dirname "$0")"
# Cargar las funciones comunes (martha_init, martha_up, martha_run...).
source docker/lib.sh
# Preparar docker compose en modo simulación. $1 = la primera palabra que escribiste
# (por ejemplo "cpu" o "software"); martha_init la usa para decidir GPU / CPU.
martha_init sim "$1"

# "case" = elegir qué hacer según la primera palabra ($1).
#   exec   = reemplazar este script por el comando (al salir del comando, termina todo)
#   ;;     = fin de esa opción
case "$1" in
  # Teleop: abre el teclado dentro del contenedor ya levantado.
  teleop) martha_need_running; exec $DC exec ros bash -ic "$TELEOP_CMD" ;;
  # Shell: una terminal bash dentro del contenedor.
  shell)  martha_need_running; exec $DC exec ros bash ;;
  # Build: recompilar ros2_ws (después de cambiar código en tu PC).
  build)  martha_need_running; exec $DC exec ros bash -ic 'cd /ros2_ws && colcon build --symlink-install' ;;
  # Pruebas: pytest de cada paquete que tenga carpeta test/ (no necesitan Gazebo).
  pruebas) martha_up
          exec $DC exec ros bash -ic 'cd /ros2_ws/src && for t in */test; do echo "== ${t%/test}"; (cd "${t%/test}" && python3 -m pytest -q test) || exit 1; done' ;;
  # Stop: apaga y borra el contenedor (tu código queda, está en tu PC).
  stop)   $DC down; exit 0 ;;
  # GPU: diagnóstico de con qué está dibujando OpenGL.
  gpu)    martha_need_running; martha_gpu_info; exit 0 ;;
  # Modelo: solo RViz con el URDF, para revisar medidas y frames sin abrir Gazebo.
  modelo) martha_up
          exec $DC exec ros bash -ic 'ros2 launch andesrobot_description display.launch.py' ;;
  # Estas opciones no hacen nada aquí: siguen hacia abajo a levantar la simulación.
  ""|cpu|sin-gazebo|software|mecanum) ;;
  # Cualquier otra cosa: mostrar la ayuda (líneas 2 a 15 de este archivo) y salir con error.
  *) sed -n '2,15p' "$0"; exit 1 ;;
esac

# Por defecto se abre la ventana de Gazebo; con "sin-gazebo" no (ahorra GPU, la física igual corre).
GAZEBO_GUI=true
[ "$1" = "sin-gazebo" ] && GAZEBO_GUI=false
# Por defecto planar_move (Gazebo mueve el cuerpo directo); con "mecanum", las ruedas con rodillos.
DRIVE=planar
[ "$1" = "mecanum" ] && DRIVE=mecanum

# Levantar el contenedor y compilar si es la primera vez.
martha_up

echo
echo "[martha] Simulación arrancando (drive=$DRIVE). En otra terminal:  ./sim.sh teleop"
[ "$DRIVE" = "mecanum" ] && echo "[martha] Ojo: con mecanum el teclado todavía no mueve a Martha (falta el puente /cmd_vel -> controlador)."
echo
# Lanzar todo (ver andesrobot_gazebo/launch/sim.launch.py). Queda en primer plano: Ctrl+C lo detiene.
exec $DC exec ros bash -ic "ros2 launch andesrobot_gazebo sim.launch.py gui:=$GAZEBO_GUI drive:=$DRIVE"
