"""PRUEBA del flujo de búsqueda de objetos: hace de orquestador mientras HRI no exista.

    ros2 run andesrobot_navigation buscar_objeto sprite

Por cada sala, de la más probable a la menos (maps/casa_objetos.yaml):
  1. va al centro de la sala            /nav/navigate_to_pose   (Navegación)
  2. gira 360° despacio                 /nav/search_rotate      (Navegación)
     mientras escucha a Visión          /vision/object_detected (std_msgs/Bool, true = lo vio)
  3. si Visión lo confirma, sigue girando hasta que deja de verlo y toma el ángulo del medio de ese
     tramo como la dirección del objeto (con true/false el objeto se ve primero en el borde de la
     cámara, no al centro). Va al mueble de la sala en esa dirección; si no lo vio, a la sala
     siguiente.
  4. al terminar, lo encuentre o no, vuelve a casa (--casa, el waypoint base por defecto; --casa ''
     para quedarse donde terminó). Si lo encontró, antes espera --espera segundos junto al mueble.

Además le dice a Visión qué buscar en /vision/target_object (std_msgs/String, latched). Los nombres
de esos dos tópicos son los de la prueba: hay que confirmarlos con Visión y agregarlos al contrato.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from andesrobot_navigation import ubicaciones as U
from andesrobot_navigation.busqueda import cargar_objetos, elegir_mueble, orden_salas
from andesrobot_navigation.ppo_core import error_angular

OBJETOS = '/maps/casa_objetos.yaml'


class Buscador(Node):

    def __init__(self, topico_deteccion, topico_objeto):
        super().__init__('buscar_objeto')
        self.ir = ActionClient(self, NavigateToPose, '/nav/navigate_to_pose')
        self.girar = ActionClient(self, Spin, '/nav/search_rotate')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_objeto = self.create_publisher(String, topico_objeto, latched)
        self.create_subscription(Bool, topico_deteccion, self.on_deteccion, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.seguidas = 0          # detecciones "true" seguidas
        self.perdidas = 0          # detecciones "false" seguidas

    def on_deteccion(self, msg):
        self.seguidas = self.seguidas + 1 if msg.data else 0
        self.perdidas = 0 if msg.data else self.perdidas + 1

    def pose(self):
        try:
            tf = self.buffer.lookup_transform('map', 'base_footprint', Time(),
                                              Duration(seconds=1.0))
        except Exception:  # noqa: BLE001 - sin TF todavía
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def esperar(self, futuro, timeout=None):
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=timeout)
        return futuro.result()

    def ir_a(self, nombre, d, timeout=120.0):
        meta = NavigateToPose.Goal()
        meta.pose = PoseStamped()
        meta.pose.header.frame_id = 'map'
        meta.pose.pose.position.x, meta.pose.pose.position.y = float(d['x']), float(d['y'])
        qz, qw = U.yaw_to_quat(float(d['yaw']))
        meta.pose.pose.orientation.z, meta.pose.pose.orientation.w = qz, qw
        self.get_logger().info(f'  yendo a {nombre} ({d["x"]:.2f}, {d["y"]:.2f})')
        handle = self.esperar(self.ir.send_goal_async(meta))
        if not handle.accepted:
            return False
        resultado = handle.get_result_async()
        rclpy.spin_until_future_complete(self, resultado, timeout_sec=timeout)
        if not resultado.done():
            self.esperar(handle.cancel_goal_async())
            return False
        return resultado.result().status == 4          # SUCCEEDED

    def yaw(self):
        pose = self.pose()
        return pose[2] if pose else None

    def girar_hasta_ver(self, confirmaciones, timeout=60.0):
        """Gira 360°; devuelve hacia dónde está el objeto (yaw en map), o None si no lo vio.

        El objeto queda entre el yaw en que Visión empezó a verlo y el yaw en que dejó de verlo:
        se devuelve el del medio. Con "confirmaciones" seguidas en los dos bordes el retardo se
        compensa.
        """
        meta = Spin.Goal()
        meta.target_yaw = 2.0 * math.pi
        meta.time_allowance = Duration(seconds=timeout).to_msg()
        self.seguidas = self.perdidas = 0
        handle = self.esperar(self.girar.send_goal_async(meta))
        if not handle.accepted:
            return None
        resultado = handle.get_result_async()
        inicio = fin = None
        while rclpy.ok() and not resultado.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if inicio is None and self.seguidas >= confirmaciones:
                inicio = self.yaw()
                self.get_logger().info(f'  Visión lo empieza a ver ({math.degrees(inicio):.0f}°)')
            elif inicio is not None and self.perdidas >= confirmaciones:
                fin = self.yaw()
                self.esperar(handle.cancel_goal_async())
                break
        if inicio is None:
            return None
        if fin is None:              # lo vio hasta terminar la vuelta
            fin = self.yaw()
        return inicio + error_angular(fin, inicio) / 2.0


def main():
    ap = argparse.ArgumentParser(description='Prueba: buscar un objeto sala por sala')
    ap.add_argument('objeto', help='nombre del objeto, el mismo que usa Visión (ej. sprite)')
    ap.add_argument('--objetos', default=OBJETOS, help='yaml con salas y objetos')
    ap.add_argument('--ubicaciones', default=None, help='yaml de waypoints')
    ap.add_argument('--deteccion', default='/vision/object_detected',
                    help='tópico Bool de Visión: true = lo vio')
    ap.add_argument('--objeto-topico', default='/vision/target_object',
                    help='tópico String donde se le dice a Visión qué buscar')
    ap.add_argument('--confirmaciones', type=int, default=3,
                    help='mensajes "true" seguidos para darlo por visto')
    ap.add_argument('--casa', default='base',
                    help="waypoint al que vuelve al terminar ('' = no volver)")
    ap.add_argument('--espera', type=float, default=3.0,
                    help='segundos junto al objeto antes de volver (lo que tomaría recogerlo)')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    _, lugares = U.load(args.ubicaciones)
    salas, objetos = cargar_objetos(args.objetos)
    orden = orden_salas(args.objeto, salas, objetos)

    rclpy.init()
    nodo = Buscador(args.deteccion, args.objeto_topico)
    log = nodo.get_logger()
    for accion, nombre in ((nodo.ir, '/nav/navigate_to_pose'), (nodo.girar, '/nav/search_rotate')):
        log.info(f'Esperando {nombre}...')
        accion.wait_for_server()
    nodo.pub_objeto.publish(String(data=args.objeto))
    log.info(f'Buscando "{args.objeto}" en: {", ".join(orden)}')

    encontrado = None
    t0 = time.monotonic()
    for sala in orden:
        log.info(f'[{sala}]')
        if sala not in lugares or not nodo.ir_a(sala, lugares[sala]):
            log.warning(f'  no se pudo llegar a {sala}: siguiente sala')
            continue
        yaw = nodo.girar_hasta_ver(args.confirmaciones)
        if yaw is None:
            log.info(f'  Visión no lo vio en {sala}')
            continue
        x, y, _ = nodo.pose()
        mueble, error = elegir_mueble(x, y, yaw, salas.get(sala, []), lugares)
        log.info(f'  ¡Visión lo vio! hacia {math.degrees(yaw):.0f}°: el mueble en esa '
                 f'dirección es {mueble} (a {math.degrees(error):.0f}°)')
        if mueble and nodo.ir_a(mueble, lugares[mueble]):
            encontrado = (sala, mueble)
        break

    if encontrado:
        log.info(f'Encontrado en {time.monotonic() - t0:.0f} s: "{args.objeto}" en '
                 f'{encontrado[1]} ({encontrado[0]})')
        time.sleep(args.espera)
    else:
        log.warning(f'No se encontró "{args.objeto}" en ninguna sala')

    # Volver a casa, haya encontrado el objeto o no.
    if args.casa:
        if args.casa not in lugares:
            log.warning(f'"{args.casa}" no es un waypoint: me quedo aquí')
        elif nodo.ir_a(args.casa, lugares[args.casa]):
            log.info(f'De vuelta en {args.casa}. Tarea terminada en {time.monotonic() - t0:.0f} s')
        else:
            log.warning(f'No pude volver a {args.casa}')
    nodo.destroy_node()
    rclpy.shutdown()
    return 0 if encontrado else 2


if __name__ == '__main__':
    sys.exit(main())
