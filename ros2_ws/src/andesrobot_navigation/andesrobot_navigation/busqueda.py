"""Lógica de la búsqueda de objetos, sin ROS (se prueba con pytest).

El flujo acordado con Visión (prueba, 2026-11-05):

    1. ir al centro de la habitación donde es más probable que esté el objeto
    2. girar 360° despacio, esperando que Visión diga "lo vi" (true/false)
    3. si lo vio: ir al mueble de esa habitación que queda hacia donde miraba Martha
       si no:     siguiente habitación más probable, y repetir

Visión solo entrega true/false, no dónde está el objeto. Por eso el paso 3 usa hacia dónde miraba
Martha cuando Visión dijo "lo vi" y elige el mueble que queda en esa dirección. Si después Visión
entrega la pose del objeto, este paso se reemplaza por ir directo a ella.
"""
import math

import numpy as np
import yaml

from andesrobot_navigation.ppo_core import error_angular

# Distancia entre un waypoint de mueble y el mueble: los del rulebook se calcularon a 0.80 m de la
# cara del mueble, mirándolo (gen_casa_robocup.py). Desde el waypoint, el mueble queda adelante.
DISTANCIA_MUEBLE = 0.80


def cargar_objetos(path):
    """Devuelve (salas, objetos) de un yaml como maps/casa_objetos.yaml.

    salas:   {sala: [muebles de esa sala]}
    objetos: {objeto: [salas, de la más probable a la menos]}
    """
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    return data.get('salas', {}), data.get('objetos', {})


def orden_salas(objeto, salas, objetos):
    """Las salas en el orden en que hay que buscar: primero las del objeto, después las demás."""
    preferidas = [s for s in objetos.get(objeto, []) if s in salas]
    return preferidas + [s for s in salas if s not in preferidas]


def punto_mueble(wp, distancia=DISTANCIA_MUEBLE):
    """Dónde está el mueble: delante del waypoint, a `distancia` metros, mirando como el yaw."""
    return (wp['x'] + distancia * math.cos(wp['yaw']), wp['y'] + distancia * math.sin(wp['yaw']))


def elegir_mueble(x, y, yaw, muebles, lugares):
    """El mueble que queda más en la dirección `yaw` vista desde (x, y).

    Devuelve (nombre, error en radianes), o (None, inf) si la sala no tiene muebles.
    """
    mejor, error = None, math.inf
    for nombre in muebles:
        if nombre not in lugares:
            continue
        mx, my = punto_mueble(lugares[nombre])
        e = abs(error_angular(math.atan2(my - y, mx - x), yaw))
        if e < error:
            mejor, error = nombre, e
    return mejor, error


class AcumuladorGiro:
    """Cuánto ha girado el robot en total (rad), sumando las diferencias de yaw entre lecturas."""

    def __init__(self):
        self.total = 0.0
        self.anterior = None

    def actualizar(self, yaw):
        if self.anterior is not None:
            self.total += abs(error_angular(yaw, self.anterior))
        self.anterior = yaw
        return self.total


def hay_linea_de_vista(grilla, origen, resolucion, p0, p1, paso=0.05, margen=0.25):
    """¿Ninguna celda ocupada entre p0 y p1? Así la visión simulada no ve a través de los muros.

    grilla: matriz (filas = y, columnas = x) con True en las celdas ocupadas, fila 0 abajo como en
    un OccupancyGrid. origen: (x, y) de la celda (0, 0) en metros. La línea termina `margen` metros
    antes de p1: el objeto está sobre un mueble, que en el mapa también es una celda ocupada.
    """
    largo = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    fin = max(0.0, 1.0 - margen / largo) if largo > 0.0 else 0.0
    n = max(2, int(largo * fin / paso))
    xs = np.linspace(p0[0], p0[0] + fin * (p1[0] - p0[0]), n)
    ys = np.linspace(p0[1], p0[1] + fin * (p1[1] - p0[1]), n)
    cols = ((xs - origen[0]) / resolucion).astype(int)
    filas = ((ys - origen[1]) / resolucion).astype(int)
    dentro = (filas >= 0) & (filas < grilla.shape[0]) & (cols >= 0) & (cols < grilla.shape[1])
    return not bool(grilla[filas[dentro], cols[dentro]].any())


def en_campo_de_vision(x, y, yaw, objetivo, fov, alcance):
    """¿El objetivo está en el cono de la cámara (ángulo fov centrado en yaw) y en su alcance?"""
    dx, dy = objetivo[0] - x, objetivo[1] - y
    if math.hypot(dx, dy) > alcance:
        return False
    return abs(error_angular(math.atan2(dy, dx), yaw)) <= fov / 2
