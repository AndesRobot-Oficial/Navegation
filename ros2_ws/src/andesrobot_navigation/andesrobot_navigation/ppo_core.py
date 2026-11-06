"""Lógica de ppo_controller que no necesita ROS: cuándo llegó y el ajuste final.

La política PPO de la tesis (Mocolax/martha_nav) sabe seguir una ruta esquivando obstáculos, pero
se entrenó para llegar a 0.3 m de la meta, sin importar hacia dónde mire. Un waypoint pide más:
dónde pararse y hacia dónde mirar. Por eso el recorrido tiene dos fases:

    fase PPO       la política manda, hasta quedar a llegada_xy de la meta
    fase final     Martha es holonómica: un control simple la acomoda en x, y, yaw y termina

Lo que queda aquí es la matemática de esa fase final y del chequeo de progreso.
"""
import math


def error_angular(a: float, b: float) -> float:
    """a - b, dando la vuelta por el lado corto: siempre entre -pi y pi."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def llego(pose, meta, xy_tol: float, yaw_tol: float) -> bool:
    """¿La pose (x, y, yaw) ya está sobre la meta (x, y, yaw), en posición y en orientación?"""
    return (math.hypot(meta[0] - pose[0], meta[1] - pose[1]) <= xy_tol
            and abs(error_angular(meta[2], pose[2])) <= yaw_tol)


def comando_final(pose, meta, bloqueado=(), k_xy=1.0, k_yaw=1.0, v_max=0.15, w_max=0.5):
    """Velocidad (vx, vy, w) en el frame del robot para acomodarse sobre la meta.

    bloqueado: lados de la huella con un obstáculo encima ('front', 'rear', 'left', 'right'), los
    que entrega footprint_blocked de la tesis. No se avanza hacia esos lados, pero sí se puede
    alejar de ellos: es la misma "guarda direccional" de su planificador local.
    """
    x, y, yaw = pose
    dx, dy = meta[0] - x, meta[1] - y
    # Del frame del mapa al del robot: lo mismo que hace la política con su objetivo.
    vx = k_xy * (math.cos(yaw) * dx + math.sin(yaw) * dy)
    vy = k_xy * (-math.sin(yaw) * dx + math.cos(yaw) * dy)
    # La rapidez se recorta sin cambiar la dirección.
    rapidez = math.hypot(vx, vy)
    if rapidez > v_max:
        vx, vy = vx * v_max / rapidez, vy * v_max / rapidez
    w = max(-w_max, min(w_max, k_yaw * error_angular(meta[2], yaw)))
    if 'front' in bloqueado:
        vx = min(vx, 0.0)
    if 'rear' in bloqueado:
        vx = max(vx, 0.0)
    if 'left' in bloqueado:
        vy = min(vy, 0.0)
    if 'right' in bloqueado:
        vy = max(vy, 0.0)
    return vx, vy, w


class ChequeoProgreso:
    """Avisa si el robot lleva demasiado tiempo sin moverse.

    Es la misma regla de SimpleProgressChecker de Nav2 (config/nav2_params.yaml): tiene que salir
    de un círculo de `radio` metros cada `tiempo` segundos. Si no, el recorrido se aborta y Nav2
    prueba sus recuperaciones (limpiar costmaps, girar, esperar, retroceder).
    """

    def __init__(self, radio: float = 0.3, tiempo: float = 15.0):
        self.radio, self.tiempo = radio, tiempo
        self.referencia = None
        self.t0 = 0.0

    def reset(self):
        self.referencia = None

    def va_bien(self, x: float, y: float, t: float) -> bool:
        """False si pasaron más de `tiempo` segundos sin alejarse `radio` metros."""
        if (self.referencia is None
                or math.hypot(x - self.referencia[0], y - self.referencia[1]) > self.radio):
            self.referencia, self.t0 = (x, y), t
            return True
        return t - self.t0 <= self.tiempo
