"""Lógica pura del filtro de seguridad de Martha (sin ROS, se prueba con pytest).

Viene del filtro de Milo (milo_ws, andesrobot_safety/logic.py), con la misma separación: aquí
solo hay matemática y el nodo ROS está en safety_filter.py. Lo que cambia es la geometría: Milo
solo avanza o retrocede, así que le basta mirar adelante y atrás. Martha es holonómica (linear.y)
y puede moverse en cualquier dirección, así que el filtro mira hacia DONDE SE ESTÁ MOVIENDO.

Todo en el frame base_footprint: +X adelante, +Y izquierda, origen = centro del robot.

Vista desde arriba, Martha moviéndose en diagonal (u = dirección del movimiento):

                                   /  /
          corredor = lo que       /  /
          barre el robot al      /  /   . <- obstáculo fuera del corredor: no cuenta
          moverse en u          /  * <---- obstáculo dentro: se mide cuánto puede
                               /  /        avanzar Martha en u antes de tocarlo
                    ┌────────┐/  /
                    │ ROBOT  │  /
                    └────────┘

Solo cuentan los obstáculos DENTRO del corredor y hacia donde se mueve. Así siempre se puede
alejar de un obstáculo, y moverse a lo largo de una pared cercana sin que el filtro la frene.
"""
import math
from dataclasses import dataclass

import numpy as np

# Valores por debajo de esto se consideran cero: evita dividir por casi nada.
EPS = 1e-6


# Los valores de aquí son los por defecto; safety_filter.py los reemplaza con los de config/.
@dataclass
class SafetyParams:
    # Caja del robot en base_footprint: la huella de nav2_params.yaml (0.58 x 0.41 m, centrada).
    footprint_half_length: float = 0.29
    footprint_half_width: float = 0.205
    # Zonas, medidas desde el borde del robot y en la dirección del movimiento.
    stop_distance: float = 0.30     # [m] no se acerca más que esto (lo pide la arquitectura)
    slow_distance: float = 0.70     # [m] empieza a frenar de a poco
    side_margin: float = 0.10       # [m] holgura a cada lado del corredor
    rotation_clearance: float = 0.15  # [m] holgura mínima alrededor para girar en el lugar
    # Límites
    max_linear: float = 0.4         # [m/s] rapidez máxima, combinando linear.x y linear.y
    max_angular: float = 1.0        # [rad/s]
    # False (teleop): cualquier giro se bloquea si hay algo dentro del círculo de giro.
    # True (Nav2): solo se bloquea el giro EN EL LUGAR. Moviéndose no se bloquea: el corredor
    # sigue frenando, y Nav2 ya revisa el footprint en cada trayectoria.
    turn_check_only_in_place: bool = False


def robot_radius(p: SafetyParams) -> float:
    """Radio del círculo que barre el robot al girar en el lugar (hasta la esquina)."""
    return math.hypot(p.footprint_half_length, p.footprint_half_width)


def remove_self(pts: np.ndarray, p: SafetyParams) -> np.ndarray:
    """Descarta puntos que caen sobre el propio robot (los postes de la torre)."""
    if pts.size == 0:
        return pts
    # True para los puntos dentro de la caja del robot (+2 cm de margen).
    inside = ((np.abs(pts[:, 0]) <= p.footprint_half_length + 0.02)
              & (np.abs(pts[:, 1]) <= p.footprint_half_width + 0.02))
    # "~" invierte True/False: devolvemos solo los puntos que NO están dentro.
    return pts[~inside]


def _franja(c: np.ndarray, u: float, h: float):
    """Cuándo entra y sale un punto de la franja |x| <= h si el robot se mueve con velocidad u.

    Con el robot avanzando t metros, el punto queda (visto desde el robot) en c - t * u. Devuelve
    los t de entrada y de salida, por punto. Si u es cero el punto no se mueve en este eje: está
    dentro de la franja siempre, o no lo está nunca.
    """
    if abs(u) < EPS:
        dentro = np.abs(c) <= h
        return np.where(dentro, -np.inf, np.inf), np.where(dentro, np.inf, -np.inf)
    s = 1.0 if u > 0.0 else -1.0
    return (c - s * h) / u, (c + s * h) / u


def free_distance(pts: np.ndarray, ux: float, uy: float, p: SafetyParams) -> float:
    """Metros que puede moverse el robot en la dirección unitaria (ux, uy) antes de tocar algo.

    Se mide desde el borde del robot. Devuelve infinito si nada se cruza en el corredor.
    """
    if pts.size == 0:
        return math.inf
    x, y = pts[:, 0], pts[:, 1]
    # El corredor es la caja del robot agrandada en side_margin, pero solo de costado respecto
    # al movimiento: yendo en +x se ensancha en y; yendo en +y, en x; en diagonal, en ambos.
    hx = p.footprint_half_length + p.side_margin * abs(uy)
    hy = p.footprint_half_width + p.side_margin * abs(ux)
    # Un punto está en el corredor si existe un t > 0 en que está en las dos franjas a la vez.
    in_x, out_x = _franja(x, ux, hx)
    in_y, out_y = _franja(y, uy, hy)
    fin = np.minimum(out_x, out_y)
    en_corredor = (np.maximum(in_x, in_y) <= fin) & (fin > 0.0)
    # La distancia se mide contra la caja REAL del robot, sin la holgura. El eje que no se mueve
    # no limita (-inf): un punto en la holgura lateral también cuenta, aunque la caja real no lo
    # toque de lleno.
    t_x = (x - math.copysign(p.footprint_half_length, ux)) / ux if abs(ux) > EPS else -np.inf
    t_y = (y - math.copysign(p.footprint_half_width, uy)) / uy if abs(uy) > EPS else -np.inf
    t = np.maximum(t_x, t_y)
    # t <= 0: el punto ya quedó al costado o detrás; moverse así no lo acerca más.
    t = t[en_corredor & (t > 0.0)]
    return float(t.min()) if t.size else math.inf


def scale_for(d: float, p: SafetyParams) -> float:
    """1 = libre, 0 = parar; lineal entre stop_distance y slow_distance."""
    if d <= p.stop_distance:
        return 0.0
    if d >= p.slow_distance:
        return 1.0
    # Ej. stop 0.3, slow 0.7, d = 0.5 -> (0.5 - 0.3) / (0.7 - 0.3) = 0.5
    return (d - p.stop_distance) / (p.slow_distance - p.stop_distance)


def filter_cmd(vx: float, vy: float, w: float, pts: np.ndarray, p: SafetyParams):
    """Devuelve (vx, vy, w, estado, detalle) seguros. pts: Nx2 en base_footprint, sin el robot.

    estado es lo que se publica en /nav/safety_state: "ok", "slow" o "stop".
    detalle es un texto para la terminal, con el motivo.
    """
    estado, detalle = 'ok', ''
    # 1) Recortar a los máximos. La rapidez se recorta sin cambiar la dirección del movimiento.
    rapidez = math.hypot(vx, vy)
    if rapidez > p.max_linear:
        vx, vy = vx * p.max_linear / rapidez, vy * p.max_linear / rapidez
        rapidez = p.max_linear
    w = max(-p.max_angular, min(p.max_angular, w))

    # 2) Si se está moviendo, cuánto espacio libre hay en esa dirección.
    if rapidez > EPS:
        d = free_distance(pts, vx / rapidez, vy / rapidez, p)
        s = scale_for(d, p)
        if s < 1.0:
            estado = 'stop' if s == 0.0 else 'slow'
            detalle = 'obstáculo a %.2f m en la dirección del movimiento' % d
        # Las dos componentes se escalan igual: frena sin desviarse.
        vx, vy = vx * s, vy * s

    # 3) Giro: si hay algo dentro del círculo que barre el robot al girar (+ holgura), no dejar
    #    girar (una esquina lo golpearía).
    en_el_lugar = math.hypot(vx, vy) < 0.02
    if w != 0.0 and pts.size and (en_el_lugar or not p.turn_check_only_in_place):
        r_min = float(np.hypot(pts[:, 0], pts[:, 1]).min())
        if r_min < robot_radius(p) + p.rotation_clearance:
            w = 0.0
            estado = 'stop'
            detalle = (detalle + ' | ' if detalle else '') + 'giro bloqueado (%.2f m)' % r_min
    return vx, vy, w, estado, detalle
