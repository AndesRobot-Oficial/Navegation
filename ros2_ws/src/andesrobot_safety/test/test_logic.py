# Pruebas de la lógica del filtro de seguridad (logic.py), con scans sintéticos y sin ROS.
# Correr:  cd ros2_ws/src/andesrobot_safety && python3 -m pytest -q test
# Son las de Milo (milo_ws, test/test_logic.py) más las de moverse de lado y en diagonal.
import math

import numpy as np
import pytest

from andesrobot_safety.logic import SafetyParams, filter_cmd, free_distance, remove_self

# Parámetros por defecto (los mismos valores que config/safety.yaml).
P = SafetyParams()
HX, HY = P.footprint_half_length, P.footprint_half_width
S2 = math.sqrt(0.5)
NADA = np.zeros((0, 2))


# Crea un "muro" de prueba: n puntos en línea, todos con la misma x, con y entre y0 e y1.
def muro_x(x, y0=-1.0, y1=1.0, n=50):
    return np.column_stack((np.full(n, x), np.linspace(y0, y1, n)))


# Lo mismo, pero con todos los puntos a la misma y (un muro al costado de Martha).
def muro_y(y, x0=-1.0, x1=1.0, n=50):
    return np.column_stack((np.linspace(x0, x1, n), np.full(n, y)))


# ------------------------- Adelante y atrás: lo mismo que Milo -------------------------
def test_libre_avanza_y_satura():
    vx, vy, _, estado, _ = filter_cmd(1.0, 0.0, 0.0, NADA, P)
    assert vx == P.max_linear and vy == 0.0 and estado == 'ok'


def test_para_en_stop_distance():
    vx, _, _, estado, _ = filter_cmd(0.3, 0.0, 0.0, muro_x(HX + P.stop_distance - 0.05), P)
    assert vx == 0.0 and estado == 'stop'


def test_frena_en_zona_lenta():
    d = (P.stop_distance + P.slow_distance) / 2
    vx, _, _, estado, _ = filter_cmd(0.4, 0.0, 0.0, muro_x(HX + d), P)
    assert math.isclose(vx, 0.2, rel_tol=1e-6) and estado == 'slow'


def test_puede_retroceder_con_muro_adelante():
    vx, _, _, estado, _ = filter_cmd(-0.2, 0.0, 0.0, muro_x(HX + 0.1), P)
    assert vx == -0.2 and estado == 'ok'


def test_para_atras():
    vx, _, _, estado, _ = filter_cmd(-0.2, 0.0, 0.0, muro_x(-HX - 0.2), P)
    assert vx == 0.0 and estado == 'stop'


def test_pasa_puerta_de_1_4_m():
    # Marcos de puerta a ±0.7 m a lo largo de 2 m adelante: fuera del corredor.
    xs = np.linspace(0.5, 2.5, 40)
    pts = np.vstack([np.column_stack((xs, np.full(40, 0.7))),
                     np.column_stack((xs, np.full(40, -0.7)))])
    vx, _, _, estado, _ = filter_cmd(0.3, 0.0, 0.0, pts, P)
    assert vx == 0.3 and estado == 'ok'


def test_giro_bloqueado_si_algo_muy_cerca():
    _, _, w, estado, detalle = filter_cmd(0.0, 0.0, 0.8, np.array([[0.0, 0.45]]), P)
    assert w == 0.0 and estado == 'stop' and 'giro bloqueado' in detalle


def test_giro_permitido_con_espacio():
    _, _, w, estado, _ = filter_cmd(0.0, 0.0, 0.8, muro_x(HX + 1.2), P)
    assert w == 0.8 and estado == 'ok'


def test_ignora_postes_de_la_torre():
    # Los 4 postes de la Martha construida: x = -0.033 y -0.208, y = ±0.11.
    postes = np.array([[-0.033, 0.11], [-0.033, -0.11], [-0.208, 0.11], [-0.208, -0.11]])
    assert remove_self(postes, P).size == 0


def test_nav_curva_permitida_y_giro_en_el_lugar_no():
    pn = SafetyParams(turn_check_only_in_place=True)
    # Paredes a ±0.45 m: moviéndose sí puede corregir el rumbo...
    xs = np.linspace(-0.3, 1.0, 30)
    pts = np.vstack([np.column_stack((xs, np.full(30, 0.45))),
                     np.column_stack((xs, np.full(30, -0.45)))])
    vx, _, w, _, _ = filter_cmd(0.2, 0.0, 0.3, pts, pn)
    assert vx == 0.2 and w == 0.3
    # ...pero girar en el lugar sigue bloqueado.
    _, _, w, estado, _ = filter_cmd(0.0, 0.0, 0.5, pts, pn)
    assert w == 0.0 and estado == 'stop'


# ------------------------- De lado: lo nuevo de Martha -------------------------
@pytest.mark.parametrize('signo', [1.0, -1.0], ids=['izquierda', 'derecha'])
def test_para_de_lado(signo):
    _, vy, _, estado, _ = filter_cmd(0.0, signo * 0.2, 0.0,
                                     muro_y(signo * (HY + P.stop_distance - 0.05)), P)
    assert vy == 0.0 and estado == 'stop'


def test_frena_de_lado_sin_desviarse():
    d = (P.stop_distance + P.slow_distance) / 2
    vx, vy, _, estado, _ = filter_cmd(0.0, 0.2, 0.0, muro_y(HY + d), P)
    assert vx == 0.0 and math.isclose(vy, 0.1, rel_tol=1e-6) and estado == 'slow'


def test_de_lado_ignora_lo_que_esta_adelante():
    # Un muro pegado adelante no molesta para irse de lado, si no se cruza en el corredor.
    vx, vy, _, estado, _ = filter_cmd(0.0, 0.2, 0.0, muro_y(0.0, x0=HX + 0.2, x1=HX + 1.0), P)
    assert vx == 0.0 and vy == 0.2 and estado == 'ok'


def test_puede_alejarse_de_un_muro_lateral():
    _, vy, _, estado, _ = filter_cmd(0.0, -0.2, 0.0, muro_y(HY + 0.1), P)
    assert vy == -0.2 and estado == 'ok'


def test_avanzar_junto_a_una_pared_no_frena():
    # Pared a 15 cm del costado: avanzar es paralelo a ella, no se acerca.
    vx, _, _, estado, _ = filter_cmd(0.3, 0.0, 0.0, muro_y(HY + 0.15), P)
    assert vx == 0.3 and estado == 'ok'


# ------------------------- En diagonal -------------------------
def test_para_en_diagonal_y_mantiene_la_direccion():
    # Punto justo en la diagonal, adelante a la izquierda de la esquina del robot.
    d = P.stop_distance - 0.05
    pts = np.array([[HX + d * S2, HY + d * S2]])
    vx, vy, _, estado, _ = filter_cmd(0.2, 0.2, 0.0, pts, P)
    assert vx == 0.0 and vy == 0.0 and estado == 'stop'


def test_frena_en_diagonal_sin_desviarse():
    d = (P.stop_distance + P.slow_distance) / 2
    pts = np.array([[HX + d * S2, HY + d * S2]])
    vx, vy, _, estado, _ = filter_cmd(0.2, 0.2, 0.0, pts, P)
    assert estado == 'slow' and vx > 0.0 and math.isclose(vx, vy, rel_tol=1e-6)


def test_diagonal_ve_lo_que_adelante_y_de_lado_no_verian():
    # Punto adelante a la izquierda, fuera del corredor de avanzar y del de ir de lado,
    # pero dentro del de la diagonal: solo el filtro a 360° lo detecta.
    pts = np.array([[HX + 0.2, HY + 0.2]])
    assert filter_cmd(0.2, 0.0, 0.0, pts, P)[3] == 'ok'
    assert filter_cmd(0.0, 0.2, 0.0, pts, P)[3] == 'ok'
    assert filter_cmd(0.2, 0.2, 0.0, pts, P)[3] == 'stop'


def test_rapidez_recortada_mantiene_la_direccion():
    vx, vy, _, _, _ = filter_cmd(1.0, 1.0, 0.0, NADA, P)
    assert math.isclose(math.hypot(vx, vy), P.max_linear) and math.isclose(vx, vy)


def test_diagonal_se_acerca_a_una_pared_lateral_y_para():
    # Pared paralela a 15 cm (fuera de la holgura): avanzar está bien, irse hacia ella no.
    pared = muro_y(HY + 0.15)
    assert filter_cmd(0.2, 0.0, 0.0, pared, P)[3] == 'ok'
    assert filter_cmd(0.2, 0.2, 0.0, pared, P)[3] == 'stop'


def test_distancia_libre_sin_puntos_es_infinita():
    assert free_distance(NADA, 1.0, 0.0, P) == math.inf


# ------------------------- Barrido de todas las direcciones -------------------------
@pytest.mark.parametrize('grados', range(0, 360, 15))
def test_nunca_se_acerca_a_un_obstaculo_en_ninguna_direccion(grados):
    # Cada 15°: un punto en la dirección pedida y más cerca que stop_distance. Siempre para.
    a = math.radians(grados)
    ux, uy = math.cos(a), math.sin(a)
    # Desde el centro hasta el borde de la caja (lo que tarde en salir por x o por y) y 20 cm más.
    borde = min(HX / abs(ux) if abs(ux) > 1e-9 else math.inf,
                HY / abs(uy) if abs(uy) > 1e-9 else math.inf)
    r = borde + P.stop_distance - 0.10
    pts = np.array([[r * ux, r * uy]])
    vx, vy, _, estado, _ = filter_cmd(0.3 * ux, 0.3 * uy, 0.0, pts, P)
    assert vx == 0.0 and vy == 0.0 and estado == 'stop'
