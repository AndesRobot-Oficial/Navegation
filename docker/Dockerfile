# Martha (AndesRobot) — ROS 2 Humble + Gazebo Classic 11 sobre Ubuntu 22.04 (jammy)
#
# Es la misma imagen de milo_ws (alejapinzonf/milo_ws) más lo que Martha necesita y Milo no: así
# los dos robots corren sobre la misma plataforma y unificar después es sumar estas líneas.
#
# Un Dockerfile es la "receta" de la imagen: cada instrucción (FROM, RUN, COPY...) crea una capa.
# Docker guarda las capas en caché: si una línea no cambia, la reutiliza y no la vuelve a hacer.
# Por eso lo pesado (instalar paquetes) va primero y lo que cambia seguido va al final.
# (Los comentarios no cuentan para la caché: agregarlos no obliga a reconstruir.)

# Imagen base oficial de Open Robotics: Ubuntu 22.04 + ROS 2 Humble + RViz y herramientas de escritorio.
FROM osrf/ros:humble-desktop

# ARG = variables que solo existen mientras se construye la imagen.
# compose.yaml les pasa tu UID/GID reales (de lib.sh) para crear un usuario igual al tuyo.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
# ENV = variable que queda en la imagen. "noninteractive" evita que apt haga preguntas
# (por ejemplo la zona horaria) y se quede esperando una respuesta que nadie va a dar.
ENV DEBIAN_FRONTEND=noninteractive

# Paquetes de ROS y herramientas (los mismos de milo_ws, en el mismo orden para compartir caché).
#   gazebo-ros-pkgs        Gazebo Classic 11 + plugins ROS (planar_move, lidar, IMU simulados)
#   xacro                  procesa los .xacro del URDF (macros, variables, includes)
#   robot-state-publisher  publica los TF del robot a partir del URDF + /joint_states
#   joint-state-publisher(-gui)  posiciones de articulaciones (con sliders) para display.launch.py
#   rviz2                  visualizador 3D
#   slam-toolbox           SLAM 2D: arma el mapa con el lidar + odometría
#   nav2-map-server        map_saver_cli: guarda el mapa en .pgm + .yaml
#   laser-filters          filtra el scan (saca del scan los postes de la torre de Martha)
#   teleop-twist-keyboard  manejar con el teclado (con Shift, Martha se mueve de lado)
#   ros2-control / ros2-controllers  controller_manager y los controladores de ruedas
#   gazebo-ros2-control    ros2_control dentro de Gazebo (ruedas mecanum con rodillos)
#   libboost-system-dev    puerto serie para drivers en C++
#   git                    para clonar el driver del lidar
#   colcon                 la herramienta que compila el workspace
#   mesa-utils             glxinfo (diagnóstico de GPU: ./sim.sh gpu)
#   sudo, bash-completion, nano: comodidades dentro del contenedor
# "--no-install-recommends" = solo lo necesario (imagen más chica).
# "rm -rf /var/lib/apt/lists/*" borra los índices de apt descargados (no se necesitan después).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-xacro \
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher \
      ros-humble-joint-state-publisher-gui \
      ros-humble-rviz2 \
      ros-humble-slam-toolbox \
      ros-humble-nav2-map-server \
      ros-humble-laser-filters \
      ros-humble-teleop-twist-keyboard \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-gazebo-ros2-control \
      libboost-system-dev \
      git \
      python3-colcon-common-extensions \
      mesa-utils \
      sudo bash-completion nano \
    && rm -rf /var/lib/apt/lists/*

# Navegación autónoma (Nav2) — capa aparte para no rehacer la de arriba (la más pesada).
#   navigation2            AMCL, planner, controller (DWB/MPPI), behaviors, bt_navigator...
#   nav2-bringup           árboles de comportamiento por defecto y ejemplos
#   nav2-simple-commander  API Python (BasicNavigator) para mandar metas desde un script
#   twist-mux              elige entre teclado y Nav2 antes del filtro de seguridad
#   python3-scipy/pil      para regenerar la casa (tools/gen_casa_robocup.py)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-nav2-simple-commander \
      ros-humble-twist-mux \
      python3-scipy python3-pil \
    && rm -rf /var/lib/apt/lists/*

# vcstool: descarga varios repositorios a la vez a partir de un archivo .repos.
RUN apt-get update && apt-get install -y --no-install-recommends python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Lo que Martha necesita y milo_ws no trae (capa aparte: lo de arriba queda igual a Milo).
#   robot-localization        ekf_node = odom_ekf_node del contrato (ruedas + IMU -> /base/odom)
#   mecanum-drive-controller  cinemática mecanum de ros2_control (ruedas con rodillos en Gazebo)
#   python3-serial            abrir el puerto serie del ESP32 a mano (pruebas y diagnóstico; en
#                             operación normal lo abre el micro-ROS agent, ver martha.repos)
# El entrenamiento del PPO (torch, stable-baselines3, gymnasium) NO va aquí: pesa varios GB y el
# robot no lo necesita, porque la política exportada (.npz) corre solo con numpy.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-robot-localization \
      ros-humble-mecanum-drive-controller \
      python3-serial \
    && rm -rf /var/lib/apt/lists/*

# Drivers externos: el del RPLIDAR de Slamtec (también maneja el A2M8 de Martha) y el micro-ROS
# agent (puente entre el ESP32 y ROS 2), listados con su versión exacta en martha.repos: así todos los PCs compilan exactamente lo
# mismo, y las versiones se cambian en un solo archivo.
# Pasos: copiar martha.repos a la imagen, descargar los repos en /opt/martha_drivers/src, cargar
# ROS ("." = source en sh), compilarlos en modo Release (optimizado) y sin sus pruebas
# (BUILD_TESTING=OFF: no las corremos y piden paquetes de lint que la imagen no trae), y borrar
# los temporales.
# Queda instalado en /opt/martha_drivers/install (entrypoint.sh y .bashrc lo cargan).
COPY martha.repos /opt/martha_drivers/martha.repos
RUN mkdir -p /opt/martha_drivers/src && cd /opt/martha_drivers \
 && vcs import src < martha.repos \
 && . /opt/ros/humble/setup.sh \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -rf build log

# Usuario con tu mismo UID/GID (los archivos de build no quedan de root en tu PC)
# 1) Si ya existe un grupo con tu GID, se renombra a "ros"; si no, se crea.
# 2) Si ya existe un usuario con tu UID (la imagen base a veces trae uno), se renombra a "ros" y
#    se le cambia el home a /home/ros; si no, se crea con bash como terminal.
# 3) Se agrega a los grupos: video (GPU), sudo (administrador) y dialout (puertos serie /dev/tty*).
# 4) sudo sin contraseña dentro del contenedor (solo dentro del contenedor, no en tu PC).
RUN if getent group ${USER_GID} >/dev/null; then \
        groupmod -n ${USERNAME} $(getent group ${USER_GID} | cut -d: -f1); \
    else groupadd --gid ${USER_GID} ${USERNAME}; fi \
 && if getent passwd ${USER_UID} >/dev/null; then \
        usermod -l ${USERNAME} -d /home/${USERNAME} -m $(getent passwd ${USER_UID} | cut -d: -f1); \
    else useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; fi \
 && usermod -aG video,sudo,dialout ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

# Copiar el script de arranque a la imagen y darle permiso de ejecución.
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Desde aquí todo corre como el usuario "ros" (no root), parado en /ros2_ws
# (que compose.yaml conecta con martha_ws/ros2_ws de tu PC).
USER ${USERNAME}
WORKDIR /ros2_ws

# Configurar ~/.bashrc del usuario: cada terminal nueva dentro del contenedor carga ROS, Gazebo,
# el driver del lidar y nuestro workspace (si ya está compilado), y tiene dos atajos:
#   cb      = compilar el workspace y recargarlo
#   teleop  = teclado que publica en /cmd_vel (mientras no exista el filtro de seguridad)
# ~/.gazebo se crea vacía para que el volumen gazebo_cache quede con dueño "ros".
# ~/.sudo_as_admin_successful apaga el aviso "To run a command as administrator..." de Ubuntu.
RUN mkdir -p ~/.gazebo && touch ~/.sudo_as_admin_successful \
 && echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
 && echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc \
 && echo 'source /opt/martha_drivers/install/setup.bash' >> ~/.bashrc \
 && echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> ~/.bashrc \
 && echo "alias cb='cd /ros2_ws && colcon build --symlink-install && source install/setup.bash'" >> ~/.bashrc \
 && echo "alias teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'" >> ~/.bashrc

# Al arrancar el contenedor: correr entrypoint.sh (carga ROS) y después CMD.
# "sleep infinity" = no hacer nada y quedarse vivo; los scripts entran con "docker compose exec".
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
