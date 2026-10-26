"""Mandar a Martha a uno o varios waypoints con nombre, con la acción /nav/navigate_to_pose.

Es lo mismo que hará task_orchestrator_node (Interacción): pedir la acción del contrato con una
pose en el frame map. Navegación decide cómo llegar.

Ejemplos (con la navegación corriendo: ./sim.sh nav):
  ros2 run andesrobot_navigation ir_a --lista
  ros2 run andesrobot_navigation ir_a sofa
  ros2 run andesrobot_navigation ir_a sofa dining_table base     # uno después del otro
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from andesrobot_navigation import ubicaciones as U

# Nombre de la acción en el contrato de la arquitectura (la atiende bt_navigator de Nav2).
ACCION = '/nav/navigate_to_pose'


class IrA(Node):
    def __init__(self):
        super().__init__('ir_a')
        self.cliente = ActionClient(self, NavigateToPose, ACCION)
        self.ultimo_aviso = None

    def ir(self, frame, d, timeout):
        """Manda la meta y espera el resultado. Devuelve True si Martha llegó."""
        meta = NavigateToPose.Goal()
        meta.pose = PoseStamped()
        meta.pose.header.frame_id = frame
        meta.pose.header.stamp = self.get_clock().now().to_msg()
        meta.pose.pose.position.x = float(d['x'])
        meta.pose.pose.position.y = float(d['y'])
        qz, qw = U.yaw_to_quat(float(d['yaw']))
        meta.pose.pose.orientation.z, meta.pose.pose.orientation.w = qz, qw

        # Mandar la meta: Nav2 la acepta o la rechaza (por ejemplo si todavía no está activo).
        envio = self.cliente.send_goal_async(meta, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, envio)
        handle = envio.result()
        if not handle.accepted:
            self.get_logger().error(f'Nav2 rechazó la meta {d["name"]}')
            return False

        # Esperar el resultado. Mientras, on_feedback avisa cuánto falta.
        resultado = handle.get_result_async()
        rclpy.spin_until_future_complete(self, resultado, timeout_sec=timeout)
        if not resultado.done():
            self.get_logger().warn(f'{d["name"]}: se acabó el tiempo ({timeout:.0f} s)')
            rclpy.spin_until_future_complete(self, handle.cancel_goal_async())
            return False
        # status 4 = SUCCEEDED (action_msgs/GoalStatus)
        return resultado.result().status == 4

    def on_feedback(self, msg):
        # Avisar cada medio metro, no en cada mensaje (llegan muchos por segundo).
        falta = msg.feedback.distance_remaining
        if self.ultimo_aviso is None or abs(falta - self.ultimo_aviso) >= 0.5:
            self.ultimo_aviso = falta
            self.get_logger().info(f'  faltan {falta:.2f} m')


def main():
    ap = argparse.ArgumentParser(description='Ir a waypoints con nombre (/nav/navigate_to_pose)')
    ap.add_argument('lugares', nargs='*', help='nombres o prefijos (sofa, din, ...)')
    ap.add_argument('--lista', action='store_true', help='mostrar los waypoints y salir')
    ap.add_argument('--ubicaciones', default=None, help='otro .yaml de waypoints')
    ap.add_argument('--timeout', type=float, default=120.0, help='s máximo por waypoint')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    frame, locs = U.load(args.ubicaciones)
    if not locs:
        print(f'No hay waypoints en {args.ubicaciones or U.DEFAULT_PATH}. Se anotan a mano: '
              'ver docs/navegacion/README.md')
        return 1
    if args.lista or not args.lugares:
        for n, d in sorted(locs.items()):
            print(f'  {n:16s} x={d["x"]:6.2f}  y={d["y"]:6.2f}  '
                  f'yaw={math.degrees(d["yaw"]):5.0f}°')
        return 0
    try:
        plan = [U.resolve(n, locs) for n in args.lugares]
    except KeyError as e:
        print(e)
        return 1

    rclpy.init()
    nodo = IrA()
    nodo.get_logger().info(f'Esperando la acción {ACCION}...')
    nodo.cliente.wait_for_server()
    llegadas = 0
    for i, d in enumerate(plan):
        nodo.get_logger().info(f'[{i + 1}/{len(plan)}] Yendo a {d["name"]} '
                               f'({d["x"]:.2f}, {d["y"]:.2f})')
        nodo.ultimo_aviso = None
        ok = nodo.ir(frame, d, args.timeout)
        llegadas += ok
        nodo.get_logger().info(f'  {d["name"]}: {"llegó" if ok else "NO llegó"}')
    nodo.get_logger().info(f'Listo: {llegadas}/{len(plan)} waypoints alcanzados')
    nodo.destroy_node()
    rclpy.shutdown()
    return 0 if llegadas == len(plan) else 2


if __name__ == '__main__':
    sys.exit(main())
