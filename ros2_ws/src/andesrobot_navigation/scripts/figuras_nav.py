#!/usr/bin/env python3
"""Figuras del informe de navegación (docs/navegacion/img/).

Copiado de milo_ws (andesrobot_navigation/scripts/figuras_nav.py), con la figura de la casa y los
lugares. Lo que cambia para Martha está marcado con "MARTHA:".

Todo sale de los mismos datos que usa el robot: la casa del generador
(andesrobot_gazebo/tools/gen_casa_robocup.py) y los lugares anotados (maps/casa_lugares.yaml).
Si cambias algo de eso, vuelve a correr este script.

Uso (en tu PC o en ./sim.sh shell; necesita numpy, matplotlib y pyyaml):
  python3 ros2_ws/src/andesrobot_navigation/scripts/figuras_nav.py
"""

import importlib.util
import math
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.patches as mp  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, '..', '..'))
REPO = os.path.abspath(os.path.join(SRC, '..', '..'))
OUT = os.path.join(REPO, 'docs', 'navegacion', 'img')
GEN = os.path.join(SRC, 'andesrobot_gazebo', 'tools', 'gen_casa_robocup.py')
# MARTHA: los mapas y lugares viven en martha_ws/maps/ (en Milo, andesrobot_slam/maps/).
LUGARES = os.path.join(REPO, 'maps', 'casa_lugares.yaml')

# MARTHA: altura del plano del lidar (base_footprint -> laser en martha.urdf.xacro). El generador
# trae la de Milo (LIDAR_Z = 0.333).
LIDAR_Z = 0.231
# MARTHA: huella de nav2_params.yaml, centrada en base_footprint (0.58 x 0.41 m).
FOOTPRINT = [(0.29, 0.205), (0.29, -0.205), (-0.29, -0.205), (-0.29, 0.205)]

AZUL, NARANJO, GRIS, ROJO, VERDE = '#2f6fd6', '#e8822a', '#8a8a8a', '#c8402c', '#3a9a4a'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.titlesize': 11,
                     'axes.spines.top': False, 'axes.spines.right': False})


def cargar_generador():
    spec = importlib.util.spec_from_file_location('gen_casa', GEN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def lugares():
    with open(LUGARES) as fh:
        d = yaml.safe_load(fh) or {}
    d = d.get('ubicaciones', d)
    return {k: v for k, v in d.items() if isinstance(v, dict)}


def guardar(fig, nombre):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, nombre)
    fig.savefig(p, dpi=160, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print('  ', os.path.relpath(p, REPO))


def dibujar_prim(ax, g, p, lidar_z=None, **kw):
    """Dibuja una primitiva del generador (en coordenadas de mundo)."""
    cz = p['c'][2]
    hz = (p['size'][2] if p['kind'] == 'box' else p['h']) / 2
    ve_lidar = lidar_z is not None and cz - hz <= lidar_z <= cz + hz
    x, y = g.to_world(p['c'][0], p['c'][1])
    color = '#2b2b2b' if ve_lidar else '#c9c9c9'
    if p['kind'] == 'cyl':
        ax.add_patch(mp.Circle((x, y), p['r'], color=color, lw=0, **kw))
    else:
        sx, sy = p['size'][0], p['size'][1]
        t = matplotlib.transforms.Affine2D().rotate_around(x, y, p['yaw']) + ax.transData
        r = mp.Rectangle((x - sx / 2, y - sy / 2), sx, sy, color=color, lw=0, **kw)
        r.set_transform(t)
        ax.add_patch(r)


def flecha_pose(ax, x, y, yaw, color, largo=0.55, **kw):
    ax.annotate('', xy=(x + largo * math.cos(yaw), y + largo * math.sin(yaw)), xytext=(x, y),
                arrowprops=dict(arrowstyle='-|>', color=color, lw=2.2, mutation_scale=14), **kw)
    ax.plot(x, y, 'o', color=color, ms=6, zorder=5)


def huella(ax, x, y, yaw, color, **kw):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(x + a * c - b * s, y + a * s + b * c) for a, b in FOOTPRINT]
    ax.add_patch(mp.Polygon(pts, closed=True, fill=False, ec=color, lw=1.3, **kw))


# ------------------------------------------------------------------------------------------
def fig_casa_lugares(g):
    locs = lugares()
    fig, ax = plt.subplots(figsize=(8.4, 7.6))
    walls = g.wall_prims()
    for p in walls:
        dibujar_prim(ax, g, p, lidar_z=LIDAR_Z)
    for f in g.FURN:
        for p in g.furn_prims(f):
            dibujar_prim(ax, g, p, lidar_z=LIDAR_Z)
    h = g.L / 2
    ax.add_patch(mp.Rectangle((-h, -h), g.L, g.L, color='#f6ead2', zorder=-1, lw=0))
    # nombres de las salas
    q = g.L / 4  # centro de cada cuarto de la casa
    for nombre, (x, y) in (('Laundry', (-q, q + 1.8)), ('Bedroom', (q + 0.9, q + 1.8)),
                           ('Living room', (-q, -0.75)), ('Kitchen', (q + 1.3, -0.75))):
        ax.text(x, y, nombre, ha='center', color='#9a7a50', fontsize=12, fontweight='bold',
                alpha=0.85)
    # partida
    sx, sy = g.to_world(*g.SPAWN_UV)
    huella(ax, sx, sy, 0.0, ROJO, ls='--')
    ax.text(sx, sy - 0.55, 'partida\nde Martha', color=ROJO, ha='center', va='top', fontsize=9)
    # lugares anotados (MARTHA: letra más chica, son 28 en vez de 6)
    for n, d in locs.items():
        x, y, yaw = float(d['x']), float(d['y']), float(d.get('yaw', 0.0))
        huella(ax, x, y, yaw, AZUL, alpha=0.7)
        flecha_pose(ax, x, y, yaw, AZUL)
        ax.text(x + 0.18, y + 0.22, n, color=AZUL, fontsize=7, fontweight='bold',
                bbox=dict(fc='white', ec='none', alpha=0.75, pad=1.2))
    ax.set_aspect('equal')
    ax.set_xlim(-h - 1.6, h + 0.5)
    ax.set_ylim(-h - 0.5, h + 0.5)
    ax.set_xlabel('x del mapa [m]')
    ax.set_ylabel('y del mapa [m]')
    ax.grid(alpha=0.25)
    ax.set_title(f'Casa RoboCup@Home y los lugares de Martha ({len(locs)} en casa_lugares.yaml)')
    leyenda = [mp.Patch(color='#2b2b2b', label=f'lo que ve el lidar (a {LIDAR_Z} m)'),
               mp.Patch(color='#c9c9c9', label='cubiertas de mesas/sillas (el lidar no las ve)'),
               plt.Line2D([], [], color=AZUL, marker='o', lw=2, label='lugar: pose (x, y) + yaw'),
               plt.Line2D([], [], color=ROJO, ls='--', lw=1.3, label='huella de Martha al partir')]
    ax.legend(handles=leyenda, loc='upper center', bbox_to_anchor=(0.5, -0.09), ncol=2,
              fontsize=8.5, frameon=False)
    guardar(fig, 'fig_casa_lugares.png')


def main():
    print('Figuras en docs/navegacion/img/:')
    g = cargar_generador()
    fig_casa_lugares(g)


if __name__ == '__main__':
    main()
