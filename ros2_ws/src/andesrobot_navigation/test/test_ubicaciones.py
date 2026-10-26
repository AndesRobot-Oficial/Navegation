"""Pruebas de los waypoints (no necesitan Gazebo ni ROS, solo el mapa y el yaml).

Correr dentro del contenedor:
    cd /ros2_ws/src/andesrobot_navigation && python3 -m pytest -q test
"""
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from andesrobot_navigation import ubicaciones as U

# martha_ws/maps/: en el contenedor está montada en /maps; en tu PC, 4 carpetas más arriba.
MAPS = Path('/maps') if Path('/maps/casa.yaml').exists() \
    else Path(__file__).resolve().parents[4] / 'maps'

# Huella de Martha (la caja de contacto del URDF y el footprint de nav2_params.yaml), en metros.
LARGO, ANCHO = 0.58, 0.41


def leer_pgm(path):
    """Lee un .pgm binario (P5) como matriz de numpy, fila 0 = arriba de la imagen."""
    data = Path(path).read_bytes()
    campos, i = [], 0
    while len(campos) < 4:  # P5, ancho, alto, valor máximo (puede haber comentarios con #)
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            i = data.index(b'\n', i)
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        campos.append(data[i:j])
        i = j
    ancho, alto = int(campos[1]), int(campos[2])
    return np.frombuffer(data[i + 1:i + 1 + ancho * alto], np.uint8).reshape(alto, ancho)


def test_formato_simple(tmp_path):
    f = tmp_path / 'lugares.yaml'
    f.write_text('sofa: {x: 1.0, y: 2.0, yaw: 0.5}\nmesa: {x: -1, y: 0}\n')
    frame, locs = U.load(f)
    assert frame == 'map'
    assert locs['sofa']['name'] == 'sofa' and locs['mesa']['yaw'] == 0.0


def test_sin_archivo_no_hay_lugares(tmp_path):
    # Un mapa recién guardado todavía no tiene <mapa>_lugares.yaml: la navegación igual arranca.
    assert U.load(str(tmp_path / 'nuevo_lugares.yaml')) == ('map', {})


def test_resolve_por_nombre_y_prefijo():
    locs = {'dining_table': {'name': 'dining_table'}, 'table': {'name': 'table'},
            'base': {'name': 'base'}}
    assert U.resolve('Dining Table', locs)['name'] == 'dining_table'
    assert U.resolve('din', locs)['name'] == 'dining_table'
    assert U.resolve('table', locs)['name'] == 'table'  # exacto gana al prefijo
    with pytest.raises(KeyError):
        U.resolve('cocina', locs)


def test_yaw_a_cuaternion():
    z, w = U.yaw_to_quat(math.pi / 2)
    assert math.isclose(z, math.sqrt(0.5)) and math.isclose(w, math.sqrt(0.5))


_, LUGARES = U.load(MAPS / 'casa_lugares.yaml')


@pytest.mark.parametrize('nombre', sorted(LUGARES))
def test_martha_cabe_en_cada_waypoint(nombre):
    # Si la huella de Martha toca una pared u obstáculo del mapa en la pose del waypoint, Nav2
    # nunca va a poder dejarla ahí (el controlador revisa el footprint completo).
    meta = yaml.safe_load((MAPS / 'casa.yaml').read_text())
    img = leer_pgm(MAPS / meta['image'])
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    ocupado = img < 255 * (1 - meta['occupied_thresh'])  # negate: 0 -> negro = ocupado
    d = LUGARES[nombre]
    c, s = math.cos(d['yaw']), math.sin(d['yaw'])
    # Recorrer la huella cada 2 cm y pasar cada punto del frame del robot al mapa (fila, columna).
    for lx in np.arange(-LARGO / 2, LARGO / 2 + 1e-9, 0.02):
        for ly in np.arange(-ANCHO / 2, ANCHO / 2 + 1e-9, 0.02):
            x, y = d['x'] + c * lx - s * ly, d['y'] + s * lx + c * ly
            col, fila = int((x - ox) / res), img.shape[0] - 1 - int((y - oy) / res)
            assert 0 <= fila < img.shape[0] and 0 <= col < img.shape[1], \
                f'{nombre} queda fuera del mapa'
            assert not ocupado[fila, col], \
                f'la huella de Martha en {nombre} toca un obstáculo en ({x:.2f}, {y:.2f})'
