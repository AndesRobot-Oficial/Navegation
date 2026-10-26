"""Pruebas del modo mapeo (no necesitan Gazebo ni ROS corriendo, solo leer los archivos).

Revisan que slam_toolbox use los nombres del contrato: si alguien cambia un tópico o un frame
aquí y no en la navegación, el mapa sale vacío o torcido sin ningún error claro.

Correr dentro del contenedor:
    cd /ros2_ws/src/andesrobot_slam && python3 -m pytest -q test
"""
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
NAV = PKG.parent / 'andesrobot_navigation'


def params():
    data = yaml.safe_load((PKG / 'config' / 'slam_toolbox_mapping.yaml').read_text())
    return data['slam_toolbox']['ros__parameters']


def test_nombre_del_nodo_coincide_con_el_launch():
    # Si no coinciden, ROS ignora el .yaml entero y slam_toolbox corre con sus valores de fábrica.
    launch = (PKG / 'launch' / 'mapping.launch.py').read_text()
    assert "name='slam_toolbox'" in launch
    assert params()


def test_frames_y_scan_del_contrato():
    p = params()
    assert p['base_frame'] == 'base_footprint'
    assert p['odom_frame'] == 'odom'
    assert p['map_frame'] == 'map'
    # El scan sin los postes de la torre, el mismo que usa la navegación (AMCL y costmaps).
    assert p['scan_topic'] == '/nav/scan_filtered'
    assert p['mode'] == 'mapping'


def test_el_filtro_es_el_de_la_navegacion():
    # El mapeo y la navegación incluyen el mismo filtro_torre.launch.py, y su nombre de nodo
    # coincide con la primera línea de filtro_torre.yaml (si no, el filtro no borra nada).
    assert 'filtro_torre.launch.py' in (PKG / 'launch' / 'mapping.launch.py').read_text()
    assert 'filtro_torre.launch.py' in (NAV / 'launch' / 'navigation.launch.py').read_text()
    filtro = yaml.safe_load((NAV / 'config' / 'filtro_torre.yaml').read_text())
    assert "name='filtro_torre'" in (NAV / 'launch' / 'filtro_torre.launch.py').read_text()
    assert 'filtro_torre' in filtro


def test_el_mapa_sale_en_nav_map():
    launch = (PKG / 'launch' / 'mapping.launch.py').read_text()
    assert "('/map', '/nav/map')" in launch


def test_rango_del_lidar():
    p = params()
    assert 0 < p['min_laser_range'] < p['max_laser_range'] <= 12.0   # el A2M8 llega a 12 m
