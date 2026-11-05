"""Pruebas de la parte del PPO que no necesita Gazebo.

La lógica de ppo_core.py se prueba siempre. Las de la política necesitan el paquete martha_nav de
este workspace, ya compilado: sin él, se saltan.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from andesrobot_navigation.ppo_core import ChequeoProgreso, comando_final, error_angular, llego

# policies/ de la raíz del repo: en el contenedor está montada en /policies; en tu PC, más arriba.
POLITICA = Path('/policies/martha_holo.npz')
if not POLITICA.is_file():
    POLITICA = Path(__file__).resolve().parents[4] / 'policies' / 'martha_holo.npz'


# ------------------------- El ajuste final -------------------------
def test_error_angular_da_la_vuelta_por_el_lado_corto():
    assert math.isclose(error_angular(3.0, -3.0), 6.0 - 2 * math.pi)
    assert math.isclose(error_angular(0.5, 0.2), 0.3)


def test_llego_pide_posicion_y_orientacion():
    meta = (1.0, 1.0, 0.0)
    assert llego((1.05, 1.0, 0.1), meta, 0.12, 0.15)
    assert not llego((1.2, 1.0, 0.0), meta, 0.12, 0.15)
    assert not llego((1.0, 1.0, 1.0), meta, 0.12, 0.15)


def test_comando_final_va_hacia_la_meta_en_el_frame_del_robot():
    # Mirando a +y, una meta en +y queda adelante: linear.x.
    vx, vy, _ = comando_final((0.0, 0.0, math.pi / 2), (0.0, 0.1, math.pi / 2))
    assert math.isclose(vx, 0.1, abs_tol=1e-9) and abs(vy) < 1e-9
    # Mirando a +x, la misma meta queda a la izquierda: linear.y, de lado.
    vx, vy, _ = comando_final((0.0, 0.0, 0.0), (0.0, 0.1, 0.0))
    assert abs(vx) < 1e-9 and math.isclose(vy, 0.1, abs_tol=1e-9)


def test_comando_final_respeta_los_limites():
    vx, vy, w = comando_final((0.0, 0.0, 0.0), (3.0, 4.0, 3.0))
    assert math.isclose(math.hypot(vx, vy), 0.15) and math.isclose(vx / vy, 3.0 / 4.0)
    assert w == 0.5


def test_comando_final_gira_por_el_lado_corto():
    _, _, w = comando_final((0.0, 0.0, 3.0), (0.0, 0.0, -3.0))
    assert w > 0.0


def test_comando_final_no_avanza_hacia_un_lado_bloqueado_pero_si_se_aleja():
    assert comando_final((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), {'front'})[0] == 0.0
    assert comando_final((0.0, 0.0, 0.0), (-0.1, 0.0, 0.0), {'front'})[0] < 0.0
    assert comando_final((0.0, 0.0, 0.0), (0.0, 0.1, 0.0), {'left'})[1] == 0.0
    assert comando_final((0.0, 0.0, 0.0), (0.0, -0.1, 0.0), {'left'})[1] < 0.0


# ------------------------- El chequeo de progreso -------------------------
def test_progreso_avisa_si_no_se_mueve():
    c = ChequeoProgreso(radio=0.3, tiempo=15.0)
    assert c.va_bien(0.0, 0.0, 0.0)
    assert c.va_bien(0.1, 0.0, 14.0)
    assert not c.va_bien(0.1, 0.0, 16.0)


def test_progreso_moverse_reinicia_el_tiempo():
    c = ChequeoProgreso(radio=0.3, tiempo=15.0)
    assert c.va_bien(0.0, 0.0, 0.0)
    assert c.va_bien(0.5, 0.0, 14.0)
    assert c.va_bien(0.5, 0.0, 28.0)


# ------------------------- La política de la tesis -------------------------
def test_la_politica_esta_en_el_paquete():
    assert POLITICA.is_file()


def nucleo():
    pytest.importorskip('martha_nav')
    from martha_nav.robots import ROBOTS
    from martha_nav.ros.numpy_policy import NumpyPolicy
    from martha_nav.ros.policy_core import PolicyCore
    modelo = NumpyPolicy(str(POLITICA))
    assert modelo.settings['robot'] == 'martha_tower' and modelo.settings['action_dim'] == 3
    return PolicyCore(modelo, action_dim=3, target=modelo.settings['target'],
                      robot=ROBOTS['martha_tower'])


def escena_libre():
    pytest.importorskip('martha_nav')
    from martha_nav.sim2d.planner import Path as Ruta
    angulos = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    return Ruta(np.array([[0.0, 0.0], [3.0, 0.0]])), np.full(360, 7.5), angulos


def test_la_politica_entrega_velocidades_dentro_de_los_limites():
    core = nucleo()
    ruta, rangos, angulos = escena_libre()
    vx, vy, w, _info = core.compute(ruta, (0.0, 0.0, 0.0), rangos, angulos, (0.0, 0.0, 0.0))
    assert -core.robot.v_reverse <= vx <= core.robot.v_max
    assert abs(vy) <= core.robot.v_lateral and abs(w) <= core.robot.w_max


def test_la_politica_avanza_si_la_ruta_esta_libre_adelante():
    core = nucleo()
    ruta, rangos, angulos = escena_libre()
    velocidad = (0.0, 0.0, 0.0)
    for _ in range(10):
        vx, vy, w, _info = core.compute(ruta, (0.0, 0.0, 0.0), rangos, angulos, velocidad)
        velocidad = (vx, vy, w)
    assert vx > 0.05
