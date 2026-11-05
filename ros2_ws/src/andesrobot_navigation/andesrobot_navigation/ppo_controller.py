"""El PPO de la tesis como controlador local de la navegación: atiende la acción /follow_path.

Es la pieza que decide las velocidades, la misma que hace DWB en controller_server. bt_navigator
sigue pidiendo la ruta a NavFn y sigue mandando "sigue esta ruta" por /follow_path, sin saber quién
la atiende. Por eso /nav/navigate_to_pose, ir_a, los waypoints y AMCL no cambian.

La lógica viene de la tesis (el paquete martha_nav, una copia en este workspace) y la política de
policies/martha_holo.npz: exportada a .npz, corre solo con numpy, sin PyTorch.

Entradas:  acción /follow_path (nav2_msgs/FollowPath)   la ruta que hay que seguir, en frame map
           /nav/scan (sensor_msgs/LaserScan)            el scan SIN filtrar: la política se entrenó
                                                        viendo los postes de la torre
           /base/odom (nav_msgs/Odometry)               la velocidad medida
           TF map -> base_footprint                     dónde está el robot (AMCL)
Salida:    /nav/cmd_vel_planner (geometry_msgs/Twist)   va a twist_mux, como lo que publica Nav2

A diferencia de DWB, no pasa por velocity_smoother: la política ya aprendió con los límites y el
retardo del robot, y agregarle un filtro cambiaría la dinámica con que se entrenó.
"""
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from martha_nav.robots import ROBOTS
from martha_nav.ros.common import yaw_of
from martha_nav.ros.numpy_policy import NumpyPolicy
from martha_nav.ros.policy_core import PolicyCore, footprint_blocked
from martha_nav.ros.scan_adapter import scan_to_arrays
from martha_nav.sim2d.planner import Path
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from andesrobot_navigation.ppo_core import ChequeoProgreso, comando_final, llego

ANTIGUO = 0.3         # s; con sensores más viejos que esto, el robot se detiene (como la tesis)


class Recorrido:
    """Lo que hace falta recordar de una meta de /follow_path."""

    def __init__(self, handle, ruta, meta):
        self.handle = handle
        self.ruta = ruta                  # martha_nav Path, o None si la ruta venía vacía
        self.meta = meta                  # (x, y, yaw) del último punto de la ruta
        self.fase = 'ppo'                 # 'ppo' -> 'final'
        self.t_final = 0.0                # cuándo empezó la fase final
        self.fin = threading.Event()      # avisa a execute_callback que ya se decidió el resultado
        self.resultado = None             # 'succeed', 'abort' o 'canceled'


class PpoController(Node):

    def __init__(self):
        super().__init__('ppo_controller')
        politica = self.declare_parameter('politica', '').value
        if not politica:
            raise RuntimeError('falta el parámetro "politica": la ruta al .npz')
        # Dónde deja la política a Martha y qué pide un waypoint para darlo por alcanzado.
        self.llegada_xy = self.declare_parameter('llegada_xy', 0.30).value    # del entrenamiento
        self.xy_tol = self.declare_parameter('xy_tol', 0.12).value            # igual que con DWB
        self.yaw_tol = self.declare_parameter('yaw_tol', 0.15).value          # igual que con DWB
        self.tiempo_final = self.declare_parameter('tiempo_final', 15.0).value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value

        modelo = NumpyPolicy(politica)
        ajustes = modelo.settings
        self.robot = ROBOTS[ajustes['robot']]
        self.core = PolicyCore(modelo,
                               lookahead=self.declare_parameter('lookahead', 1.5).value,
                               action_dim=ajustes['action_dim'], target=ajustes['target'],
                               robot=self.robot)
        self.dim = self.core.action_dim
        self.progreso = ChequeoProgreso(self.declare_parameter('radio_progreso', 0.3).value,
                                        self.declare_parameter('tiempo_progreso', 15.0).value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.scan = self.t_scan = self.t_odom = None
        self.velocidad = (0.0,) * (self.dim if self.dim == 3 else 2)
        self.lock = threading.Lock()
        self.actual = None                # el Recorrido que se está siguiendo
        self.recorridos = {}              # por id de meta, para execute_callback

        # El control corre en su propio grupo; los hilos de la acción esperan sin bloquearlo.
        control = MutuallyExclusiveCallbackGroup()
        accion = ReentrantCallbackGroup()
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data,
                                 callback_group=control)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data,
                                 callback_group=control)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # 10 Hz: la frecuencia con que se entrenó la política.
        self.create_timer(0.1, self.tick, callback_group=control)
        self.servidor = ActionServer(self, FollowPath, 'follow_path',
                                     execute_callback=self.ejecutar,
                                     goal_callback=lambda _meta: GoalResponse.ACCEPT,
                                     cancel_callback=lambda _handle: CancelResponse.ACCEPT,
                                     handle_accepted_callback=self.aceptar,
                                     callback_group=accion)
        self.get_logger().info(f"PPO listo: {politica}, robot {ajustes['robot']}, "
                               f'acción {self.dim}D, llegada {self.llegada_xy} m')

    # ---------------- Entradas ----------------
    def on_scan(self, msg):
        self.scan = scan_to_arrays(msg)
        self.t_scan = self.get_clock().now()

    def on_odom(self, msg):
        t = msg.twist.twist
        self.velocidad = ((t.linear.x, t.linear.y, t.angular.z) if self.dim == 3
                          else (t.linear.x, t.angular.z))
        self.t_odom = self.get_clock().now()

    def fresco(self, t):
        return t is not None and self.get_clock().now() - t < Duration(seconds=ANTIGUO)

    def pose(self):
        try:
            tf = self.buffer.lookup_transform(self.map_frame, self.base_frame, Time(),
                                              Duration(seconds=0.05))
        except Exception:  # noqa: BLE001 - al arrancar es normal que todavía no haya TF
            return None
        return (tf.transform.translation.x, tf.transform.translation.y,
                yaw_of(tf.transform.rotation))

    # ---------------- La acción /follow_path ----------------
    def aceptar(self, handle):
        """bt_navigator replanifica cada segundo: cada ruta nueva reemplaza a la anterior."""
        poses = handle.request.path.poses
        puntos = [(p.pose.position.x, p.pose.position.y) for p in poses]
        ruta = Path(np.array(puntos)) if len(puntos) >= 2 else None
        meta = None
        if puntos:
            meta = (*puntos[-1], yaw_of(poses[-1].pose.orientation))
        nuevo = Recorrido(handle, ruta, meta)
        with self.lock:
            viejo = self.actual
            if viejo is not None:
                # Misma meta: es un replan. La memoria de la política y la fase se conservan.
                if meta is not None and viejo.meta is not None and np.allclose(meta, viejo.meta):
                    nuevo.fase, nuevo.t_final = viejo.fase, viejo.t_final
                else:
                    self.empezar_de_cero()
                self.cerrar(viejo, 'abort')
            else:
                self.empezar_de_cero()
            self.actual = nuevo
            self.recorridos[bytes(handle.goal_id.uuid)] = nuevo
        handle.execute()

    def empezar_de_cero(self):
        self.core.reset()
        self.progreso.reset()

    def ejecutar(self, handle):
        """Corre en un hilo aparte: espera a que tick() decida y entrega el resultado."""
        recorrido = self.recorridos.pop(bytes(handle.goal_id.uuid))
        while rclpy.ok() and not recorrido.fin.wait(timeout=0.2):
            pass
        if recorrido.resultado == 'succeed':
            handle.succeed()
        elif recorrido.resultado == 'canceled':
            handle.canceled()
        else:
            handle.abort()
        return FollowPath.Result()

    def cerrar(self, recorrido, resultado):
        recorrido.resultado = resultado
        recorrido.fin.set()

    def terminar(self, resultado, motivo=''):
        """Termina el recorrido actual y deja al robot quieto."""
        self.pub.publish(Twist())
        if motivo:
            self.get_logger().warning(motivo)
        self.cerrar(self.actual, resultado)
        self.actual = None

    # ---------------- Control ----------------
    def tick(self):
        with self.lock:
            if self.actual is not None:
                self.paso(self.actual)

    def paso(self, r):
        # Sin un recorrido activo no se publica nada: las recuperaciones de Nav2 usan este tópico.
        if r.handle.is_cancel_requested:
            return self.terminar('canceled')
        if r.meta is None:
            return self.terminar('abort', 'ruta vacía')
        if not self.fresco(self.t_scan) or not self.fresco(self.t_odom):
            self.pub.publish(Twist())
            return self.get_logger().warning('sensores viejos', throttle_duration_sec=2.0)
        pose = self.pose()
        if pose is None:
            self.pub.publish(Twist())
            return self.get_logger().warning('sin TF map -> base', throttle_duration_sec=2.0)
        rangos, angulos = self.scan
        ahora = self.get_clock().now().nanoseconds * 1e-9
        distancia = float(np.hypot(r.meta[0] - pose[0], r.meta[1] - pose[1]))
        if r.fase == 'ppo' and (distancia <= self.llegada_xy or r.ruta is None):
            r.fase, r.t_final = 'final', ahora

        cmd = Twist()
        if r.fase == 'final':
            if distancia > 2.0 * self.llegada_xy:
                return self.terminar('abort', f'lejos de la meta: {distancia:.2f} m')
            if llego(pose, r.meta, self.xy_tol, self.yaw_tol):
                return self.terminar('succeed')
            if ahora - r.t_final > self.tiempo_final:
                # Quedó dentro de lo que la política considera llegar: no vale la pena abortar.
                return self.terminar('succeed', f'ajuste final incompleto: a {distancia:.2f} m')
            bloqueado = footprint_blocked(rangos, angulos, self.robot)
            cmd.linear.x, cmd.linear.y, cmd.angular.z = comando_final(pose, r.meta, bloqueado)
        else:
            if not self.progreso.va_bien(pose[0], pose[1], ahora):
                return self.terminar('abort', 'sin progreso: Nav2 probará sus recuperaciones')
            *velocidades, _info = self.core.compute(r.ruta, pose, rangos, angulos, self.velocidad)
            cmd.linear.x = float(velocidades[0])
            if self.dim == 3:
                cmd.linear.y = float(velocidades[1])
            cmd.angular.z = float(velocidades[-1])
        self.pub.publish(cmd)

        feedback = FollowPath.Feedback()
        feedback.distance_to_goal = distancia
        feedback.speed = float(np.hypot(cmd.linear.x, cmd.linear.y))
        r.handle.publish_feedback(feedback)


def main():
    rclpy.init()
    nodo = PpoController()
    # Varios hilos: mientras ejecutar() espera el resultado, el control tiene que seguir corriendo.
    ejecutor = MultiThreadedExecutor(num_threads=4)
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
