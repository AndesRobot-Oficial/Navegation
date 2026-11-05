# Política PPO de navegación local

`martha_holo.npz` es la política de la tesis de Nicolás (PPO, aprendizaje por refuerzo) para Martha
con la torre de aluminio. Es un **planificador local**: sigue una ruta global y decide la velocidad
`(vx, vy, w)` en cada paso a partir del LiDAR. Corre solo con NumPy (sin PyTorch).

- Origen: `martha_tower_holo_s0`, mejor punto (1.2M pasos), copiado de
  [martha_nav/policies](https://github.com/Mocolax/martha_nav/tree/main/policies).
- Perfil del robot: `martha_tower` (va dentro del `.npz`, en `settings`). El cargador lo compara con
  el perfil actual y falla si no coincide.
- Resultados (sección 10 de
  [resultados.md](https://github.com/Mocolax/martha_nav/blob/main/docs/resultados.md)): en Gazebo,
  laboratorio, 87 % de llegadas y 1 % de choques en 100 episodios; puntos fijos, 88.9 % y 1.1 %
  en 90. **Aún no se ha probado en el robot real.**

## Contrato de la política

| | Qué es |
|---|---|
| Entrada (98 valores en [-1, 1]) | 90 sectores de 4° del LiDAR (mínimo por sector, codificado `d/(d+1)`, los sectores tapados por los postes de la torre con valor fijo) · *carrot* `(dist/3 m, ángulo/π)` en el marco del robot · velocidad medida `(vx/0.35, vy/0.25, w/0.8)` · acción anterior |
| Salida (3 valores en [-1, 1]) | `vx` × 0.35 m/s (× 0.15 hacia atrás) · `vy` × 0.25 m/s · `w` × 0.8 rad/s |
| *Carrot* | punto de la ruta global a 1.5 m por delante del robot, saltando los que caen a < 0.4 m de un obstáculo |

## Cómo está integrada

La corre `andesrobot_navigation/ppo_controller.py`, el controlador local de `./sim.sh nav`. Reutiliza
el código de abajo, copiado sin cambios en el paquete `ros2_ws/src/martha_nav`. La carpeta se monta
en el contenedor como `/policies`.

En `martha_nav` ya está el código, sin ROS y probado:

| Archivo de `martha_nav` | Qué hace |
|---|---|
| `martha_nav/ros/numpy_policy.py` | carga el `.npz` y calcula la acción |
| `martha_nav/ros/policy_core.py` | un paso de control: ruta + pose + scan → velocidad, con freno por lado |
| `martha_nav/sim2d/observation.py` | arma la observación y convierte la acción en velocidad |

El nodo original de la tesis (`ppo_local_planner.py`) usa nombres sin prefijo (`/scan`, `/odom`,
`/plan` → `/cmd_vel`). `ppo_controller` usa los del contrato de la arquitectura: lee `/nav/scan`,
`/base/odom` y la ruta global de Nav2, y publica en `/nav/cmd_vel_planner` (no en `/cmd_vel`, que es
solo del filtro de seguridad).
