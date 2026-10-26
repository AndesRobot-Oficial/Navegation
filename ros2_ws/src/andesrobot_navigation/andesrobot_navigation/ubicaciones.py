"""Carga los waypoints con nombre (maps/casa_lugares.yaml). Sin ROS: se prueba solo con pytest.

Cada waypoint es una pose en el frame "map": dónde se para Martha (x, y en metros) y hacia dónde
mira (yaw en radianes, 0 = hacia +x del mapa). Se anotan a mano después de mapear, por ejemplo
llevando a Martha al lugar y leyendo su pose con  ros2 run tf2_ros tf2_echo map base_footprint
"""

import math
import os

import yaml


# Archivo de waypoints al lado del mapa. martha_ws/maps/ de tu PC = /maps en el contenedor.
DEFAULT_PATH = '/maps/casa_lugares.yaml'


def load(path=None):
    """Devuelve (frame_id, {nombre: dict(x, y, yaw, name)}).

    Acepta dos formatos (los mismos de milo_ws):
      - simple, una línea por lugar:   sofa: {x: -3.15, y: -4.03, yaw: -1.55}
      - completo: frame_id: map  +  ubicaciones: {sofa: {...}, ...}  (room, id opcionales)

    Si el archivo no existe (un mapa recién hecho, todavía sin lugares anotados) devuelve
    ningún lugar en vez de fallar: así la navegación igual arranca con ese mapa.
    """
    path = path or DEFAULT_PATH
    if not os.path.isfile(path):
        return 'map', {}
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    if 'ubicaciones' in data:
        locs = data['ubicaciones']
    else:
        locs = {k: v for k, v in data.items() if isinstance(v, dict)}
        data = {}
    for name, d in locs.items():
        d.setdefault('yaw', 0.0)
        d['name'] = name
    return data.get('frame_id', 'map'), locs


def resolve(name, locs):
    """Busca por nombre exacto o por prefijo único ("din" -> dining_table)."""
    key = str(name).strip().lower().replace(' ', '_')
    if key in locs:
        return locs[key]
    matches = [d for n, d in locs.items() if n.startswith(key)]
    if len(matches) == 1:
        return matches[0]
    raise KeyError(
        f'El lugar "{name}" no existe o es ambiguo. Opciones: {", ".join(sorted(locs))}')


def yaw_to_quat(yaw):
    """(z, w) del cuaternión para un giro yaw en el plano (x = y = 0)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)
