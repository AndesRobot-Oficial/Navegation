"""safety_filter_node de la arquitectura: /nav/cmd_vel_mux + /nav/scan -> /cmd_vel.

Igual para simulación y para Martha física. Frena si hay obstáculos a menos de `stop_distance`
en la dirección en que se mueve, incluyendo de lado (linear.y); deja alejarse o girar para salir.
Viene del filtro de Milo (milo_ws, andesrobot_safety/safety_filter.py).

Este archivo es la parte "ROS" (topics, parámetros, timers). La matemática está en logic.py.

Entradas:  /nav/scan (sensor_msgs/LaserScan)          lidar
           /nav/cmd_vel_mux (geometry_msgs/Twist)     la velocidad que eligió el mux
Salidas:   /cmd_vel (Twist)                           lo que realmente se le manda a la base
           /nav/safety_state (std_msgs/String)        "ok", "slow" o "stop"
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from andesrobot_safety.logic import SafetyParams, filter_cmd, remove_self


class SafetyFilter(Node):

    def __init__(self):
        # El nombre del contrato: `ros2 topic info /cmd_vel` debe mostrar a este nodo.
        super().__init__('safety_filter_node')
        # Un parámetro por cada campo de SafetyParams; ROS los reemplaza con los de config/.
        d = SafetyParams()
        for name, default in vars(d).items():
            self.declare_parameter(name, default)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('rate', 20.0)          # Hz a los que publica /cmd_vel
        self.declare_parameter('scan_timeout', 0.5)   # s sin lidar -> parar
        self.declare_parameter('cmd_timeout', 0.0)    # 0 = mantener el último comando (teclado)

        self.p = SafetyParams(**{k: self.get_parameter(k).value for k in vars(d)})
        self.base_frame = self.get_parameter('base_frame').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # TF: dónde está el lidar respecto al centro del robot.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.laser_tf = None             # (tx, ty, yaw) laser -> base, fijo

        self.points = np.zeros((0, 2))   # últimos puntos del lidar (en base_footprint)
        self.last_scan = None            # cuándo llegó el último scan
        self.cmd = Twist()               # último comando (empieza en 0)
        self.last_cmd = None             # cuándo llegó
        self.last_state = ''             # último estado publicado (para no repetirlo)

        # "scan", "cmd_vel_in" y "cmd_vel" se remapean en launch/safety.launch.py.
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel_in', self.on_cmd, 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        # transient_local = "latched": quien se suscriba después igual recibe el último estado,
        # aunque el estado se publique solo cuando cambia.
        estado_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(String, '/nav/safety_state', estado_qos)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.get_logger().info(
            'Filtro de seguridad: stop %.2f m, slow %.2f m, v_max %.2f m/s'
            % (self.p.stop_distance, self.p.slow_distance, self.p.max_linear))

    def on_cmd(self, msg):
        # Solo se guarda: el que decide qué mandar a la base es step(), con el timer.
        self.cmd = msg
        self.last_cmd = self.get_clock().now()

    def on_scan(self, msg):
        # La primera vez: dónde está el lidar respecto a base_footprint (no cambia).
        if self.laser_tf is None:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, msg.header.frame_id, Time(), timeout=Duration(seconds=0.0))
            except Exception:
                # Todavía no llegan los TF (pasa al arrancar): ignorar este scan.
                return
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.laser_tf = (t.transform.translation.x, t.transform.translation.y, yaw)
        # De distancia y ángulo en el frame del lidar a (x, y) en base_footprint.
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r >= max(msg.range_min, 0.05)) & (r <= msg.range_max)
        tx, ty, yaw = self.laser_tf
        ang = a[ok] + yaw
        pts = np.column_stack((tx + r[ok] * np.cos(ang), ty + r[ok] * np.sin(ang)))
        # Sacar lo que cae sobre el propio robot (los postes de la torre) y guardar.
        self.points = remove_self(pts, self.p)
        self.last_scan = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        # Twist() vacío = velocidad 0 (lo que se manda si algo está mal).
        out = Twist()
        if (self.last_scan is None
                or (now - self.last_scan).nanoseconds * 1e-9 > self.scan_timeout):
            # Sin lidar el filtro está ciego: no moverse.
            state, detalle = 'stop', 'sin lidar'
        elif (self.cmd_timeout > 0.0 and self.last_cmd is not None
              and (now - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout):
            # Dejaron de mandar comandos: quedarse quieto, sin que sea una alarma.
            state, detalle = 'ok', ''
        else:
            vx, vy, w, state, detalle = filter_cmd(
                self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z, self.points, self.p)
            out.linear.x, out.linear.y, out.angular.z = vx, vy, w
        # Publicar SIEMPRE (aunque sea 0): si este nodo se cae, la base deja de recibir y se
        # detiene sola (el watchdog de 0.5 s del contrato).
        self.pub_cmd.publish(out)
        if state != self.last_state:
            self.pub_state.publish(String(data=state))
            if state == 'stop':
                self.get_logger().warn('stop: ' + detalle)
            self.last_state = state


def main():
    rclpy.init()
    node = SafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Al cerrar: mandar velocidad 0 por si acaso, destruir el nodo y apagar ROS.
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
