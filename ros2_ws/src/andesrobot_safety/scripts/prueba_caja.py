#!/usr/bin/env python3
"""Prueba en Gazebo del criterio de NAV-11: Martha no choca una caja, en ninguna dirección.

Pone una caja adelante, otra a la izquierda y otra en diagonal, y manda a Martha contra cada una
por /nav/cmd_vel_mux. Por cada caja revisa con /base/odom (en Gazebo es la pose real) que se haya
detenido sin tocarla, y que /nav/safety_state haya pasado a "stop".

Necesita un mundo vacío con Martha en el origen, y el filtro corriendo. Dentro de ./sim.sh shell:
  ros2 launch andesrobot_gazebo sim.launch.py gui:=false rviz:=false \
      world:=/usr/share/gazebo-11/worlds/empty.world x:=0.0 y:=0.0 yaw:=0.0
  ros2 launch andesrobot_safety safety.launch.py use_sim_time:=true
  python3 /ros2_ws/src/andesrobot_safety/scripts/prueba_caja.py
"""
import math
import sys
import time

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

# Caja de 40 cm de lado, tan alta que cruza el plano del lidar (a 0.231 m).
LADO = 0.40
SDF = f"""<sdf version="1.6"><model name="caja"><static>true</static><link name="l">
<collision name="c"><geometry><box><size>{LADO} {LADO} 0.6</size></box></geometry></collision>
<visual name="v"><geometry><box><size>{LADO} {LADO} 0.6</size></box></geometry></visual>
</link></model></sdf>"""

# La huella de Martha (nav2_params.yaml) y la velocidad con que va contra la caja.
HX, HY = 0.29, 0.205
RAPIDEZ = 0.3
S2 = math.sqrt(0.5)
# nombre, dirección del movimiento, centro de la caja (lejos del corredor de las otras).
CASOS = [
    ('adelante', (1.0, 0.0), (2.0, 0.0)),
    ('de lado', (0.0, 1.0), (0.0, 2.0)),
    ('en diagonal', (S2, S2), (2.2, 2.2)),
]


def holgura(x, y, cx, cy):
    """Distancia entre la huella de Martha en (x, y) y la caja en (cx, cy). <= 0 = chocó."""
    dx = abs(cx - x) - HX - LADO / 2
    dy = abs(cy - y) - HY - LADO / 2
    if dx <= 0.0 and dy <= 0.0:
        return max(dx, dy)
    return math.hypot(max(dx, 0.0), max(dy, 0.0))


class Prueba(Node):

    def __init__(self):
        super().__init__('prueba_caja')
        self.pose = None
        self.estado = ''
        self.vistos = set()
        self.pub = self.create_publisher(Twist, '/nav/cmd_vel_mux', 10)
        self.create_subscription(Odometry, '/base/odom', self.on_odom, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/nav/safety_state', self.on_estado, qos)
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')

    def on_odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_estado(self, msg):
        self.estado = msg.data
        self.vistos.add(msg.data)

    def esperar(self, condicion, timeout):
        fin = time.monotonic() + timeout
        while time.monotonic() < fin:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condicion():
                return True
        return False

    def poner_caja(self, nombre, x, y):
        req = SpawnEntity.Request(name=nombre, xml=SDF)
        req.initial_pose = Pose()
        req.initial_pose.position.x, req.initial_pose.position.y = x, y
        req.initial_pose.position.z = 0.3
        fut = self.spawn.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result() is not None and fut.result().success

    def mandar(self, vx, vy, segundos):
        """Publica una velocidad a 20 Hz durante un tiempo (como lo haría el mux)."""
        msg = Twist()
        msg.linear.x, msg.linear.y = vx, vy
        fin = time.monotonic() + segundos
        while time.monotonic() < fin:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def volver_al_origen(self):
        """Regresa alejándose de la caja: el filtro tiene que dejarlo."""
        for _ in range(400):
            x, y = self.pose
            if math.hypot(x, y) < 0.03:
                break
            k = min(1.0, 0.3 / math.hypot(x, y))
            self.mandar(-x * k, -y * k, 0.1)
        self.mandar(0.0, 0.0, 0.3)


def main():
    rclpy.init()
    nodo = Prueba()
    if not nodo.spawn.wait_for_service(timeout_sec=30.0):
        print('No está /spawn_entity: ¿está corriendo Gazebo?')
        return 1
    if not nodo.esperar(lambda: nodo.pose is not None and nodo.estado == 'ok', 60.0):
        print('No llegan /base/odom o /nav/safety_state = ok: ¿está corriendo el filtro?')
        return 1
    for i, (nombre, _, (cx, cy)) in enumerate(CASOS):
        if not nodo.poner_caja(f'caja_{i}', cx, cy):
            print(f'No se pudo poner la caja {nombre}')
            return 1
    # Dar tiempo a que el lidar vea las cajas.
    nodo.mandar(0.0, 0.0, 1.0)

    fallas = 0
    for nombre, (ux, uy), (cx, cy) in CASOS:
        nodo.vistos.clear()
        x0, y0 = nodo.pose
        # Va contra la caja mucho más tiempo del que necesitaría para chocarla.
        nodo.mandar(RAPIDEZ * ux, RAPIDEZ * uy, 3.0 * math.hypot(cx, cy) / RAPIDEZ)
        x, y = nodo.pose
        h = holgura(x, y, cx, cy)
        avance = math.hypot(x - x0, y - y0)
        ok = h > 0.0 and 'stop' in nodo.vistos and avance > 0.5
        fallas += not ok
        resultado = 'OK' if ok else 'FALLA'
        print(f'{nombre:12s} {resultado:5s} avanzó {avance:.2f} m, quedó a {h:.2f} m de la caja, '
              f'estados vistos: {sorted(nodo.vistos)}')
        nodo.volver_al_origen()

    nodo.destroy_node()
    rclpy.shutdown()
    print('Resultado:', 'OK' if not fallas else f'{fallas} FALLAS')
    return 1 if fallas else 0


if __name__ == '__main__':
    sys.exit(main())
