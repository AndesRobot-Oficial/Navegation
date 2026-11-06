"""Girar en el lugar, despacio, mientras Visión busca: la acción /nav/search_rotate.

Es la parte de Navegación del flujo de búsqueda: el orquestador pide girar (por ejemplo 360°) y
cancela el giro apenas Visión confirma que vio el objeto. Gira con /nav/cmd_vel_look, la entrada
del mux para "mirar" (prioridad 20, sobre Nav2), así que no pelea con la navegación.

Usa el tipo nav2_msgs/action/Spin mientras no exista el paquete de interfaces del equipo (NAV-01):
    meta:       target_yaw (rad, con signo: + = antihorario) y time_allowance
    feedback:   angular_distance_traveled (rad)
    resultado:  total_elapsed_time

Parámetros: velocidad (rad/s, 0.3 por defecto: unos 21 s por vuelta, a acordar con Visión según
su cámara y sus cuadros por segundo).
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import Spin
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from andesrobot_navigation.busqueda import AcumuladorGiro


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GirarBuscando(Node):

    def __init__(self):
        super().__init__('girar_buscando')
        self.velocidad = self.declare_parameter('velocidad', 0.3).value
        self.yaw = None
        grupo = ReentrantCallbackGroup()
        self.create_subscription(Odometry, '/base/odom', self.on_odom, qos_profile_sensor_data,
                                 callback_group=grupo)
        self.pub = self.create_publisher(Twist, '/nav/cmd_vel_look', 10)
        ActionServer(self, Spin, '/nav/search_rotate', execute_callback=self.ejecutar,
                     goal_callback=lambda _meta: GoalResponse.ACCEPT,
                     cancel_callback=lambda _handle: CancelResponse.ACCEPT,
                     callback_group=grupo)
        self.get_logger().info(f'Listo: /nav/search_rotate a {self.velocidad} rad/s')

    def on_odom(self, msg):
        self.yaw = yaw_de(msg.pose.pose.orientation)

    def ejecutar(self, handle):
        meta = handle.request
        objetivo = abs(meta.target_yaw)
        sentido = 1.0 if meta.target_yaw >= 0.0 else -1.0
        limite = Duration.from_msg(meta.time_allowance).nanoseconds * 1e-9 or math.inf
        giro = AcumuladorGiro()
        cmd = Twist()
        cmd.angular.z = sentido * self.velocidad
        t0 = self.get_clock().now()
        estado = 'abort'
        while rclpy.ok():
            if handle.is_cancel_requested:
                estado = 'canceled'
                break
            transcurrido = (self.get_clock().now() - t0).nanoseconds * 1e-9
            if transcurrido > limite:
                self.get_logger().warning(f'se acabó el tiempo ({limite:.0f} s) sin dar el giro')
                break
            recorrido = giro.actualizar(self.yaw) if self.yaw is not None else 0.0
            if recorrido >= objetivo:
                estado = 'succeed'
                break
            self.pub.publish(cmd)
            feedback = Spin.Feedback()
            feedback.angular_distance_traveled = float(recorrido)
            handle.publish_feedback(feedback)
            time.sleep(0.1)
        # Quieta al terminar, por cualquier motivo.
        self.pub.publish(Twist())
        resultado = Spin.Result()
        resultado.total_elapsed_time = (self.get_clock().now() - t0).to_msg()
        getattr(handle, estado)()
        return resultado


def main():
    rclpy.init()
    nodo = GirarBuscando()
    ejecutor = MultiThreadedExecutor(num_threads=3)
    ejecutor.add_node(nodo)
    try:
        ejecutor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.pub.publish(Twist())
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
