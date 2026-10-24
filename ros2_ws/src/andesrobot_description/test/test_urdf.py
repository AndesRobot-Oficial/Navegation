"""Pruebas del URDF de Martha (no necesitan Gazebo ni el grafo de ROS, solo xacro).

Correr dentro del contenedor:
    cd /ros2_ws/src/andesrobot_description && python3 -m pytest -q test
"""
import itertools
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro

URDF_DIR = Path(__file__).resolve().parents[1] / 'urdf'
CONTROLLERS = Path(__file__).resolve().parents[1] / 'config' / 'martha_controllers.yaml'


def expand(**args):
    """El URDF final (texto) con los argumentos de xacro dados."""
    mappings = {'controllers_file': str(CONTROLLERS)}
    mappings.update({k: str(v).lower() for k, v in args.items()})
    return xacro.process_file(str(URDF_DIR / 'martha.urdf.xacro'), mappings=mappings).toxml()


@pytest.mark.parametrize('path', sorted(URDF_DIR.glob('*.xacro')), ids=lambda p: p.name)
def test_sin_dos_puntos_y_espacio(path):
    # gazebo_ros2_control lee el URDF como YAML: "dos puntos + espacio" lo rompe (drive:=mecanum).
    lineas = [i + 1 for i, linea in enumerate(path.read_text().splitlines()) if ': ' in linea]
    assert not lineas, f'{path.name} tiene ": " en las líneas {lineas}'


@pytest.mark.parametrize('drive,sim,tower',
                         itertools.product(['planar', 'mecanum'], [True, False], [True, False]))
def test_frames_del_contrato(drive, sim, tower):
    texto = expand(drive=drive, sim=sim, tower=tower)
    assert ': ' not in texto
    robot = ET.fromstring(texto)
    # Solo los hijos directos de <robot>: ros2_control también tiene elementos <joint>.
    links = {link.get('name') for link in robot.findall('link')}
    hijos = {joint.find('child').get('link') for joint in robot.findall('joint')}
    # base_footprint es la raíz (nadie la tiene como hija) y laser / imu_link existen.
    assert 'base_footprint' in links and 'base_footprint' not in hijos
    assert {'base_link', 'laser', 'imu_link'} <= links
    assert any(name.startswith('tower_post') for name in links) == tower


def test_topics_del_contrato_en_simulacion():
    texto = expand(drive='planar', sim=True)
    for remap in ('~/out:=/nav/scan', '~/out:=/base/imu', 'cmd_vel:=/cmd_vel',
                  'odom:=/base/odom'):
        assert remap in texto
    assert '<robot_base_frame>base_footprint</robot_base_frame>' in texto


def test_sin_plugins_fuera_de_simulacion():
    assert '<plugin' not in expand(drive='planar', sim=False)
