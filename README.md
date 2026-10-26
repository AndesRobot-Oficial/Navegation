<div align="center">

# martha_ws

**Workspace de MARTHA, la base holonómica de AndesRobot para RoboCup@Home 2027**

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)
![Gazebo Classic 11](https://img.shields.io/badge/Gazebo-Classic_11-F58113)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04_(Docker)-E95420?logo=ubuntu&logoColor=white)

Equipos de **Navegación** y **Estructura** · Colombia

</div>

---

## Qué es esto

El workspace donde Navegación y Estructura construyen a Martha mientras se define el repositorio
general del equipo. Sigue las mismas reglas que [`milo_ws`](https://github.com/alejapinzonf/milo_ws)
(el workspace de Milo) y que la
[arquitectura ROS 2 de AndesRobot](https://alejapinzonf.github.io/Robot_arquitecture_ROS2/):

- **Mismos nombres de paquete** (`andesrobot_*`) y mismo Docker que `milo_ws`, para que unir los
  dos robots en un solo código (`robot:=milo | martha`) sea copiar archivos, no reescribirlos.
- **Mismos tópicos y frames del contrato** desde el primer día: `/cmd_vel`, `/base/odom`,
  `/base/imu`, `/nav/scan`, `base_footprint`, `laser`.
- **Simulación y robot real con el mismo código**: en Gazebo los plugins publican exactamente lo
  que después publican los drivers reales.

Martha: base con **4 ruedas mecanum** (holonómica: avanza, gira y se mueve de lado),
**RPLIDAR A2M8**, IMU, **ESP32** con 4 drivers de motor, torre de perfiles de aluminio y un
portátil arriba.

---

## Contenido

1. [Instalar (una vez por computador)](#1-instalar-una-vez-por-computador)
2. [Simulación](#2-simulación)
3. [Todos los comandos](#3-todos-los-comandos)
4. [Qué hay en el repo](#4-qué-hay-en-el-repo)
5. [El contrato: qué publica Martha](#5-el-contrato-qué-publica-martha)
6. [Estado y lo que falta](#6-estado-y-lo-que-falta)
7. [Problemas comunes](#7-problemas-comunes)

> Todo corre dentro de **Docker** (Ubuntu 22.04 + ROS 2 Humble), así que funciona igual en
> cualquier Ubuntu (también 24.04 o 26.04). Tu código vive en tu PC (`~/martha_ws/ros2_ws`) y el
> contenedor lo ve como `/ros2_ws`: editas en tu PC y compilas/corres dentro de Docker con
> `./sim.sh`.

---

## 1. Instalar (una vez por computador)

### Docker

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2 docker-buildx git x11-xserver-utils
sudo usermod -aG docker $USER
```

Después **reinicia** el computador (o cierra sesión y vuelve a entrar). Para ver si quedó bien:

```bash
docker run --rm hello-world
```

Tiene que decir `Hello from Docker!`. Si dice *permission denied*, falta reiniciar.

<details>
<summary><b>Solo si tu computador tiene tarjeta NVIDIA</b> (clic para ver)</summary>

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Sin esto igual funciona, pero Gazebo dibuja con la CPU (más lento).
</details>

### El workspace

```bash
git clone https://github.com/Mocolax/martha_ws ~/martha_ws
cd ~/martha_ws
```

> La **primera vez** que corras `./sim.sh` se demora harto (15-30 min): descarga la imagen de
> Docker y compila todo. Después parte en segundos.

> **¿Ya tienes Ubuntu 22.04 con ROS 2 Humble?** Docker igual es el camino recomendado (todos con
> lo mismo). Si prefieres nativo, instala los paquetes de `docker/Dockerfile` con `apt` y compila
> **fuera** de `ros2_ws/` para no romper el `install/` del contenedor:
> `colcon build --symlink-install --build-base /tmp/martha_build --install-base /tmp/martha_install`

---

## 2. Simulación

| Terminal | Comando | Qué hace |
|:-:|---|---|
| 1 | `./sim.sh` | Gazebo con la casa RoboCup@Home + Martha + RViz |
| 2 | `./sim.sh teleop` | manejar a Martha con el teclado |

Martha parte **afuera de la puerta del living**, mirando hacia adentro (la misma partida de Milo).

**Teclas del teleop** (la terminal del teleop tiene que estar seleccionada: haz clic en ella):

```
   u    i    o          i = adelante        , = atrás
   j    k    l          j / l = girar       k = frenar
   m    ,    .          q / z = más rápido / más lento

   Con Shift (U I O J K L M < >) Martha se mueve DE LADO: J = izquierda, L = derecha.
```

En RViz se ven el modelo, el **lidar** (puntos rojos, `/nav/scan`) y la **odometría** (flechas
amarillas, `/base/odom`).

**Revisar solo el modelo** (sin Gazebo): `./sim.sh modelo` abre RViz con Martha y sus frames.

**Apagar:** `Ctrl+C` en la terminal 1 y después `./sim.sh stop`.

### Navegación autónoma: lugares con nombre

El mapa de la casa ya está en `maps/casa.yaml` (lo hizo Milo; en la simulación coincide con el
mundo de Gazebo). Martha parte **afuera de la puerta del living**, mirando hacia adentro.
Informe completo, con la figura de todos los lugares:
[docs/navegacion/README.md](docs/navegacion/README.md).

#### Paso 1 · Anotar dónde queda cada cosa

El mapa solo sabe qué está libre y qué está ocupado; no sabe qué es "el sofá". Un **lugar** es
una pose en el mapa: `x`, `y` y hacia dónde mira Martha (`yaw`). Se anotan así, **con
`./sim.sh nav` abierto**:

```bash
./sim.sh shell
ros2 run tf2_ros tf2_echo map base_footprint
```

Lleva a Martha con el teleop (`./sim.sh teleop`) frente a cada lugar, mirándolo, y lee:

```
- Translation: [-3.147, -4.026, -0.000]          ← x, y   (los 2 primeros)
- Rotation: in RPY (radian) [0.000, -0.000, -1.551]   ← yaw (el último)
```

Anótalo en `maps/casa_lugares.yaml`, una línea por lugar:

```yaml
sofa: {x: -3.15, y: -4.03, yaw: -1.55}
dining_table: {x: 1.572, y: -2.738, yaw: 0.398}
```

> En la simulación el mapa coincide con el mundo de Gazebo, así que estas poses sirven siempre
> que vuelvas a abrir la casa.

Hoy hay **28 lugares**: los 6 que se anotaron con Milo, las 18 ubicaciones del rulebook y el
centro de cada sala (de dónde sale cada uno, en el
[informe](docs/navegacion/README.md#origen)). `./sim.sh pruebas` revisa que Martha quepa en cada
uno sin tocar el mapa.

#### Paso 2 · Mandar a Martha a un lugar

**a) A mano** (para entender qué pasa por dentro), en `./sim.sh shell`:

```bash
ros2 action send_goal /nav/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.572, y: -2.738}, orientation: {z: 0.1977, w: 0.9803}}}}" --feedback
```

Nav2 no recibe el `yaw` directo sino como cuaternión: **`z = sin(yaw/2)`, `w = cos(yaw/2)`**.
La acción lleva el nombre del contrato de la
[arquitectura](https://alejapinzonf.github.io/Robot_arquitecture_ROS2/), `/nav/navigate_to_pose`,
la misma que pedirá `task_orchestrator_node`.

**b) Por nombre:**

| Terminal | Comando | Qué hace |
|:-:|---|---|
| 1 | `./sim.sh nav` | Gazebo + AMCL + Nav2 + RViz con el mapa `casa` |
| 2 | `./sim.sh ir --lista` | ver tus lugares |
| 2 | `./sim.sh ir sofa` | ir al sofá |
| 2 | `./sim.sh ir sofa base dining_table` | varios seguidos |

También sirve **2D Goal Pose** en RViz (clic en el mapa y arrastrar para la dirección).
Si tocas el teleop le quitas el control a Nav2 mientras aprietes teclas.

<details>
<summary><b>Cómo fluyen las velocidades</b> (clic para ver)</summary>

```
Nav2 ──────── /nav/cmd_vel_planner ──┐
look_controller ─ /nav/cmd_vel_look ─┼─► twist_mux ─► /cmd_vel ─► Martha
teclado ─────────── /cmd_vel_teleop ─┘   (el teclado tiene prioridad)
```

Es la cadena de la arquitectura. Martha todavía no tiene filtro de seguridad: cuando exista, el
mux publicará en `/nav/cmd_vel_mux` y el filtro en `/cmd_vel`. El lidar pasa antes por un filtro
que borra los postes de la torre (`/nav/scan` → `/nav/scan_filtered`); Nav2 usa el modelo de
movimiento **Omni**, así que Martha puede ir de lado.
</details>

---

## 3. Todos los comandos

| Comando | Qué hace |
|---|---|
| `./sim.sh` | Gazebo (casa RoboCup@Home) + Martha + RViz |
| `./sim.sh teleop` | manejar con el teclado (con Shift, de lado) |
| `./sim.sh modelo` | solo el URDF en RViz, sin Gazebo |
| `./sim.sh pruebas` | pruebas (pytest) de todos los paquetes |
| `./sim.sh mecanum` | simulación con las ruedas mecanum y ros2_control (ver [Estado](#6-estado-y-lo-que-falta)) |
| `./sim.sh shell` | terminal dentro de Docker |
| `./sim.sh build` | recompilar si cambiaste código |
| `./sim.sh stop` | apagar el contenedor |
| `./sim.sh cpu` / `software` / `sin-gazebo` | sin NVIDIA / render por CPU / sin ventana de Gazebo |
| `./sim.sh gpu` | con qué tarjeta gráfica está dibujando |
| `./sim.sh nav [mapa]` | navegación autónoma con un mapa de `maps/` (por defecto `casa`) |
| `./sim.sh ir <lugar> [...]` | mandar a Martha a uno o varios waypoints (`--lista` para verlos) |

---

## 4. Qué hay en el repo

```
martha_ws/
├── sim.sh                     todo se lanza desde aquí
├── docker/                    imagen de Docker (la de milo_ws + lo que necesita Martha)
├── docs/navegacion/           informe de los lugares con nombre (y sus figuras)
├── maps/                      mapa de la casa (casa.pgm + casa.yaml) y sus waypoints
│                              (casa_lugares.yaml); en Docker se ve como /maps
└── ros2_ws/src/
    ├── andesrobot_description   URDF de Martha (chasis, ruedas mecanum, lidar, IMU, torre)
    ├── andesrobot_gazebo        casa RoboCup@Home (y su generador, tools/) y spawn de Martha
    └── andesrobot_navigation    Nav2 (modelo Omni), twist_mux, filtro de la torre y waypoints
```

**El URDF** (`andesrobot_description/urdf/martha.urdf.xacro`) viene de `martha_nav` (la tesis de
navegación) y representa a la Martha real. Argumentos:

| Argumento | Valores | Para qué |
|---|---|---|
| `drive` | `planar` (defecto) / `mecanum` | `planar`: Gazebo mueve el cuerpo directo con `/cmd_vel` (rápido, para navegación). `mecanum`: las 4 ruedas con rodillos y ros2_control (realista, lento) |
| `tower` | `true` (defecto) / `false` | la torre de aluminio; sus postes cruzan el plano del lidar |
| `sim` | `true` / `false` | agrega los plugins de Gazebo (`martha.sim.xacro`) |
| `lidar_samples` | `360` (defecto) | rayos por vuelta del lidar simulado |

> **Regla del URDF:** en los `.xacro` no se escribe nunca *dos puntos seguidos de un espacio*, ni
> en comentarios: con `drive:=mecanum`, `gazebo_ros2_control` lee el URDF como YAML y eso lo
> rompe. `./sim.sh pruebas` lo revisa.

**La casa** (`andesrobot_gazebo/worlds/robocup_casa.world`) es la misma de `milo_ws`: 12.6 × 12.6 m,
puertas de 1.4 m. Se genera con `andesrobot_gazebo/tools/gen_casa_robocup.py`, copiado igual
desde `milo_ws`; si cambia allá, se copian aquí el script y el mundo. Martha (huella 0.58 × 0.41 m, radio de giro 0.44 m) cabe donde cabe
Milo (radio de giro 0.59 m).

---

## 5. El contrato: qué publica Martha

Navegación y Estructura se conectan solo por estos tópicos. En Gazebo los publican los plugins;
en el robot real, los drivers. Nadie más cambia su código al pasar de uno a otro.

| Tópico / TF | Tipo | Quién lo publica | Simulación | Robot real |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` (`linear.x`, `linear.y`, `angular.z`) | Navegación (filtro de seguridad) → Estructura | `planar_move` lo recibe | puente al ESP32 |
| `/base/odom` + TF `odom → base_footprint` | `nav_msgs/Odometry` | Estructura | `planar_move` | EKF (ruedas + IMU) |
| `/base/imu` | `sensor_msgs/Imu` | Estructura | plugin IMU | IMU real |
| `/nav/scan` | `sensor_msgs/LaserScan`, frame `laser` | Navegación | plugin ray | `sllidar_ros2` (A2M8) |

Frames: `base_footprint` (suelo, raíz) → `base_link` → `laser`, `imu_link`, ruedas y torre.
`ROS_DOMAIN_ID` por defecto **12** (Martha); cada persona que simule en la misma red puede usar el
suyo: `ROS_DOMAIN_ID=31 ./sim.sh`.

---

## 6. Estado y lo que falta

**Funciona:** Docker, Martha en la casa RoboCup@Home con `planar_move`, teleop holonómico, lidar e
IMU simulados con los nombres del contrato, pruebas del URDF, navegación autónoma con Nav2 (modelo
Omni) sobre el mapa de la casa y waypoints con nombre.

**Lo que viene** (en este orden aproximado):

| Equipo | Qué | Paquete |
|---|---|---|
| Estructura | Base real: firmware del ESP32 (cinemática mecanum) y puente serie → `/cmd_vel` / `/base/odom` | `firmware/`, `andesrobot_base` |
| Estructura | EKF ruedas + IMU (`robot_localization`) → `/base/odom` | `andesrobot_base` |
| Estructura | Simulación mecanum completa: puente `/cmd_vel` → `mecanum_drive_controller` → `/base/odom` | `andesrobot_base` |
| Navegación | Driver del A2M8 real → `/nav/scan`; SLAM y un mapa propio de Martha (el de `maps/` lo hizo Milo) | `andesrobot_slam` |
| Navegación | `/nav/pose` y el servicio `/nav/set_initial_pose` del contrato (hoy AMCL parte en la pose del launch) | `andesrobot_navigation` |
| Navegación | Mapa y `base_zone` propios de Martha (la arquitectura los pide por robot; hoy `base` es la de Milo) | `maps/` |
| Navegación | Filtro de seguridad que también mire a los lados; el mux pasa a publicar `/nav/cmd_vel_mux` | `andesrobot_safety` |
| Ambos | `robot.sh` para el portátil de Martha | raíz |

---

## 7. Problemas comunes

| Problema | Solución |
|---|---|
| Gazebo o RViz se ven **negros** | `./sim.sh stop` y después `./sim.sh software` |
| `permission denied` con Docker | falta reiniciar después de `usermod -aG docker` |
| Gazebo no abre (`exit 255`) | quedó un Gazebo colgado: `./sim.sh stop` |
| El teleop no responde | haz clic en la terminal del teleop antes de apretar teclas |
| Cambié código y no se nota | `./sim.sh build` (con la simulación abierta en otra terminal) |
| Martha no se mueve de lado | el teleop necesita **Shift** apretado (`J` / `L` mayúsculas) |
| Con `./sim.sh nav`, `ir` se queda en *Esperando la acción* o Nav2 rechaza la meta | Nav2 no terminó de activarse (a veces se pierde una respuesta al arrancar, en el log sale `failed to send response`): `Ctrl+C` y otra vez `./sim.sh nav` |

---

<div align="center">
<sub>AndesRobot · RoboCup@Home 2027 · Docker y scripts adaptados de <a href="https://github.com/alejapinzonf/milo_ws">milo_ws</a></sub>
</div>
