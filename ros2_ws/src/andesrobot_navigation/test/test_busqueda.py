"""Pruebas de la lógica de búsqueda de objetos (busqueda.py), sin ROS."""
import math
from pathlib import Path

import numpy as np

from andesrobot_navigation import ubicaciones as U
from andesrobot_navigation.busqueda import (AcumuladorGiro, cargar_objetos, elegir_mueble,
                                            en_campo_de_vision, hay_linea_de_vista, orden_salas,
                                            punto_mueble)

MAPS = Path('/maps') if Path('/maps/casa_objetos.yaml').exists() \
    else Path(__file__).resolve().parents[4] / 'maps'


def test_orden_salas_primero_las_probables_y_despues_las_demas():
    salas = {'kitchen': [], 'living_room': [], 'laundry': [], 'bedroom': []}
    assert orden_salas('sprite', salas, {'sprite': ['living_room', 'kitchen']}) == \
        ['living_room', 'kitchen', 'laundry', 'bedroom']
    assert orden_salas('desconocido', salas, {}) == list(salas)


def test_los_objetos_de_la_casa_usan_salas_y_muebles_que_existen():
    salas, objetos = cargar_objetos(MAPS / 'casa_objetos.yaml')
    _, lugares = U.load(MAPS / 'casa_lugares.yaml')
    for sala, muebles in salas.items():
        assert sala in lugares, sala
        assert all(m in lugares for m in muebles), muebles
    for objeto, orden in objetos.items():
        assert all(s in salas for s in orden), objeto


def test_punto_mueble_queda_delante_del_waypoint():
    x, y = punto_mueble({'x': 1.0, 'y': 2.0, 'yaw': math.pi / 2})
    assert math.isclose(x, 1.0, abs_tol=1e-9) and math.isclose(y, 2.8)


def test_elegir_mueble_segun_hacia_donde_miraba():
    lugares = {'a': {'x': 2.0, 'y': -0.8, 'yaw': math.pi / 2},     # mueble en (2, 0): al este
               'b': {'x': -0.8, 'y': 2.0, 'yaw': 0.0}}             # mueble en (0, 2): al norte
    assert elegir_mueble(0.0, 0.0, 0.0, ['a', 'b'], lugares)[0] == 'a'
    assert elegir_mueble(0.0, 0.0, math.pi / 2, ['a', 'b'], lugares)[0] == 'b'
    assert elegir_mueble(0.0, 0.0, 0.0, [], lugares) == (None, math.inf)


def test_acumulador_suma_el_giro_aunque_pase_por_pi():
    g = AcumuladorGiro()
    for yaw in np.linspace(0.0, 2.0 * math.pi, 100):
        g.actualizar(math.atan2(math.sin(yaw), math.cos(yaw)))
    assert math.isclose(g.total, 2.0 * math.pi, rel_tol=1e-3)


def test_campo_de_vision_y_alcance():
    assert en_campo_de_vision(0.0, 0.0, 0.0, (2.0, 0.5), math.radians(60), 4.0)
    assert not en_campo_de_vision(0.0, 0.0, 0.0, (2.0, 2.0), math.radians(60), 4.0)
    assert not en_campo_de_vision(0.0, 0.0, 0.0, (5.0, 0.0), math.radians(60), 4.0)


def test_un_muro_tapa_la_linea_de_vista():
    grilla = np.zeros((40, 40), dtype=bool)
    grilla[:, 20] = True                       # muro vertical en x = 1.0 m
    assert not hay_linea_de_vista(grilla, (0.0, 0.0), 0.05, (0.5, 1.0), (1.5, 1.0))
    assert hay_linea_de_vista(grilla, (0.0, 0.0), 0.05, (0.2, 1.0), (0.8, 1.0))
    # El objeto sobre el mueble: la celda del mueble no tapa al propio objeto.
    assert hay_linea_de_vista(grilla, (0.0, 0.0), 0.05, (0.2, 1.0), (1.0, 1.0))
