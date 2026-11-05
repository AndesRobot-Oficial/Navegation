# martha_nav: copia de la lógica del PPO

**No se edita aquí.** Es una copia sin cambios de lo que necesita la política para correr en el
robot, del repo de la tesis [`Mocolax/martha_nav`](https://github.com/Mocolax/martha_nav), commit
`ea34bc4` (2026-10-30), el de la política `martha_tower_holo_s0`. Si cambia allá, se vuelve a copiar
junto con `policies/martha_holo.npz`.

Se copia, en vez de agregarlo a `docker/martha.repos`, porque `docker/` tiene que ser igual al del
repo de Estructura (ver `docker/compose.yaml`). El entrenamiento no viene aquí: corre solo con
numpy, scipy y pyyaml, que ya están en la imagen.

| Módulo | Qué hace |
|---|---|
| `martha_nav/ros/numpy_policy.py` | carga el `.npz` y calcula la acción |
| `martha_nav/ros/policy_core.py` | un paso de control, con el freno por lado |
| `martha_nav/ros/scan_adapter.py`, `ros/common.py` | del `LaserScan` a arreglos, y utilidades |
| `martha_nav/sim2d/observation.py` | arma la observación y convierte la acción en velocidad |
| `martha_nav/sim2d/planner.py`, `sim2d/dynamics.py` | la ruta, el *carrot* y el periodo de control |
| `martha_nav/robots.py`, `learning/__init__.py` | el perfil de Martha y una utilidad del modelo |

El nodo ROS que la usa es `andesrobot_navigation/ppo_controller.py`.
