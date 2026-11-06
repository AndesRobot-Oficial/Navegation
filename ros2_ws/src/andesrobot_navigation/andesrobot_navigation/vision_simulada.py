"""SOLO PARA PROBAR: hace de object_detector_node de Visión mientras no esté.

Publica /vision/object_detected (std_msgs/Bool) a 5 Hz: true si el objeto está dentro del cono de
la cámara de Martha, más cerca que el alcance y sin un muro del mapa en medio. Sabe qué objeto se
busca por /vision/target_object (std_msgs/String); si se busca otro, publica false.

    ros2 run andesrobot_navigation vision_simulada --ros-args -p objeto:=sprite -p en:=counter

Parámetros: objeto (nombre), en (mueble de casa_lugares.yaml donde está), fov (grados, 60) y
alcance (m, 4.0). No ve muebles ni la altura del objeto: solo los muros del mapa.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from andesrobot_navigation import ubicaciones as U
from andesrobot_navigation.busqueda import en_campo_de_vision, hay_linea_de_vista, punto_mueble


class VisionSimulada(Node):

    def __init__(self):
        super().__init__('vision_simulada')
        self.objeto = self.declare_parameter('objeto', 'sprite').value
        en = self.declare_parameter('en', 'counter').value
        self.fov = math.radians(self.declare_parameter('fov', 60.0).value)
        self.alcance = self.declare_parameter('alcance', 4.0).value
        _, lugares = U.load(self.declare_parameter('ubicaciones', '').value or None)
        self.punto = punto_mueble(lugares[en])
        self.buscado = None
        self.grilla = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/nav/map', self.on_mapa, latched)
        self.create_subscription(String, '/vision/target_object', self.on_buscado, latched)
        self.pub = self.create_publisher(Bool, '/vision/object_detected', 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f'"{self.objeto}" en {en} {tuple(round(v, 2) for v in self.punto)}')

    def on_mapa(self, msg):
        datos = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.grilla = datos > 50
        self.origen = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.resolucion = msg.info.resolution

    def on_buscado(self, msg):
        self.buscado = msg.data

    def tick(self):
        visto = False
        if self.grilla is not None and self.buscado == self.objeto:
            try:
                tf = self.buffer.lookup_transform('map', 'base_footprint', Time(),
                                                  Duration(seconds=0.1))
                x, y = tf.transform.translation.x, tf.transform.translation.y
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                visto = (en_campo_de_vision(x, y, yaw, self.punto, self.fov, self.alcance)
                         and hay_linea_de_vista(self.grilla, self.origen, self.resolucion,
                                                (x, y), self.punto))
            except Exception:  # noqa: BLE001 - sin TF todavía
                pass
        self.pub.publish(Bool(data=bool(visto)))


def main():
    rclpy.init()
    nodo = VisionSimulada()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
