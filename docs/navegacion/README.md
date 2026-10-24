*martha_ws · ROS 2 Humble · Gazebo classic 11 · Nav2 · Docker*

# Martha: lugares con nombre (waypoints)

Informe del equipo de **Navegación** de AndesRobot. Explica cómo se le enseñan lugares con nombre
a Martha y cómo llega sola a ellos. Sigue el mismo esquema del
[informe de navegación de Milo](https://github.com/alejapinzonf/milo_ws/blob/main/docs/navegacion/README.md)
(sección 7, *Lugares con nombre*); aquí está solo lo que cambia para Martha. Todo se probó en la
casa RoboCup@Home simulada en Gazebo; los valores salen del código y de esas pruebas.

**Alcance:** `andesrobot_navigation`, `maps/` y la casa de `andesrobot_gazebo`

| Dato | Qué es |
| ---: | --- |
| **12.6 × 12.6 m** | la casa del rulebook en Gazebo (la misma de Milo): 4 piezas, 18 muebles, puertas de 1.40 m |
| **5 cm** | tamaño de cada celda del mapa (271 × 249 celdas para la casa entera) |
| **28 lugares** | 6 anotados a mano con Milo, 18 del rulebook y el centro de las 4 salas; Martha llegó a todos con Nav2 |
| **0.58 × 0.41 m** | la huella de Martha, centrada en `base_footprint` |

## Contenido

1. [Lugares con nombre](#lugares)
2. [De dónde salen los 28 lugares](#origen)
3. [Qué revisa la prueba](#prueba)
4. [Cómo usarlo](#uso)
5. [Pendientes](#pendientes)

<a id="lugares"></a>

## 1. Lugares con nombre

> **En palabras simples:** el mapa sabe qué está libre y qué está ocupado, pero no sabe qué es "el
> sofá". Un lugar es una **pose** del mapa con un nombre: dónde pararse y hacia dónde mirar.

**Cómo se anotan.** Con la simulación abierta (`./sim.sh nav`), se lleva a Martha con el teclado
(`./sim.sh teleop`) frente a cada cosa, mirándola, y se pregunta dónde está (en `./sim.sh shell`):

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

```
- Translation: [-3.147, -4.026, -0.000]              ← x, y
- Rotation: in RPY (radian) [0.000, -0.000, -1.551]   ← el último es el yaw
```

Y se anota en `maps/casa_lugares.yaml`, una línea por lugar:

```yaml
sofa: {x: -3.15, y: -4.03, yaw: -1.55}
dining_table: {x: 1.572, y: -2.738, yaw: 0.398}
```

> En la simulación el mapa coincide con el mundo de Gazebo, así que estas poses sirven siempre
> que vuelvas a abrir la casa.

**Por qué Nav2 pide un cuaternión.** Una orientación en 3D no se puede guardar bien con tres
ángulos (hay posiciones donde se "traban"), así que ROS usa cuaterniones: cuatro números
$`(x, y, z, w)`$. Cuando el giro es solo en el plano, como el de Martha, queda:

```math
q = \big(0,\ 0,\ \sin\tfrac{\psi}{2},\ \cos\tfrac{\psi}{2}\big)
\qquad\Longleftrightarrow\qquad
\psi = \operatorname{atan2}\!\big(2(wz + xy),\ 1 - 2(y^2 + z^2)\big)
```

**Se lee así:** para mandar a Martha mirando con $`\psi = -1.55`$ rad (el sofá),
$`z = \sin(-0.775) = -0.700`$ y $`w = \cos(-0.775) = 0.714`$, que es justo lo que mostraba la línea
*Quaternion* del `tf2_echo`. La fórmula de la derecha es la vuelta: de cuaternión a yaw.
$`q`$ y $`-q`$ son el mismo giro, por eso el yaw de `outdoor` (−3.171, casi −π) puede salir con
$`(z, w) = (-0.9999, -0.0147)`$ o con $`(0.9999, 0.0147)`$.

<a id="fig-casa"></a>

![Plano de la casa visto desde arriba: cuatro piezas (Laundry arriba a la izquierda, Bedroom arriba a la derecha, Living room abajo a la izquierda, Kitchen abajo a la derecha), muros y muebles en negro, cubiertas de mesas en gris claro, y en azul los 28 lugares con la huella de Martha y una flecha hacia donde mira. A la izquierda, afuera de la puerta del living, la huella roja punteada de Martha al partir.](img/fig_casa_lugares.png)

**Figura 1.** La casa en el frame `map` y los 28 lugares de `maps/casa_lugares.yaml`. En negro, lo
que ve el lidar de Martha a 0.231 m de altura; en gris claro, las cubiertas de mesas y sillas, que
el lidar **no** ve porque pasa por debajo (solo ve las patas). Cada lugar lleva la huella de Martha
(0.58 × 0.41 m) en su pose.

**Los lugares** (`maps/casa_lugares.yaml`):

| Lugar | x [m] | y [m] | yaw [rad] | z | w |
| --- | ---: | ---: | ---: | ---: | ---: |
| `sofa` | −3.150 | −4.030 | −1.550 | −0.6997 | 0.7144 |
| `dining_table` | 1.572 | −2.738 | 0.398 | 0.1977 | 0.9803 |
| `table` | −3.943 | −4.308 | −1.828 | −0.7920 | 0.6106 |
| `base` | −2.145 | −1.770 | −1.564 | −0.7047 | 0.7095 |
| `indoor` | −6.644 | −2.153 | −0.259 | −0.1291 | 0.9916 |
| `outdoor` | −5.700 | −3.982 | −3.171 | −0.9999 | −0.0147 |
| `laundry_table` | −4.298 | 4.650 | 1.571 | 0.7072 | 0.7070 |
| `washing_machine` | −2.310 | 4.750 | 1.571 | 0.7072 | 0.7070 |
| `shelf` | −4.900 | 2.142 | −1.571 | −0.7072 | 0.7070 |
| `laundry_trash_bin` | −3.472 | 2.182 | −1.571 | −0.7072 | 0.7070 |
| `bed` | 2.840 | 1.612 | 3.142 | 1.0000 | −0.0002 |
| `bedside_table` | 0.910 | 3.895 | −1.571 | −0.7072 | 0.7070 |
| `coat_rack` | 5.143 | 4.893 | 0.785 | 0.3825 | 0.9240 |
| `tv_stand` | −4.312 | −0.358 | 1.571 | 0.7072 | 0.7070 |
| `sofa_7` | −3.150 | −4.450 | −1.571 | −0.7072 | 0.7070 |
| `coffee_table` | −0.970 | −4.855 | −1.571 | −0.7072 | 0.7070 |
| `cabinet` | 1.240 | −0.350 | 3.142 | 1.0000 | −0.0002 |
| `refrigerator` | 4.700 | −0.350 | 0.000 | 0.0000 | 1.0000 |
| `counter` | 0.940 | −4.750 | −1.571 | −0.7072 | 0.7070 |
| `sink` | 1.820 | −4.750 | −1.571 | −0.7072 | 0.7070 |
| `cooking_table` | 2.660 | −4.750 | −1.571 | −0.7072 | 0.7070 |
| `dishwasher` | 4.340 | −4.750 | −1.571 | −0.7072 | 0.7070 |
| `kitchen_trash_bin` | 4.870 | −3.360 | 0.000 | 0.0000 | 1.0000 |
| `dinner_table` | 4.056 | −3.020 | 2.356 | 0.9238 | 0.3828 |
| `laundry` | −3.080 | 3.500 | 0.000 | 0.0000 | 1.0000 |
| `bedroom` | 3.360 | 3.920 | 0.000 | 0.0000 | 1.0000 |
| `living_room` | −2.940 | −2.380 | 0.000 | 0.0000 | 1.0000 |
| `kitchen` | 1.960 | −3.780 | 1.571 | 0.7072 | 0.7070 |

**Mandar a Martha a mano**, por ejemplo a la mesa del comedor (en `./sim.sh shell`, con
`./sim.sh nav` abierto). La acción lleva el nombre del contrato de la
[arquitectura](https://alejapinzonf.github.io/Robot_arquitecture_ROS2/), `/nav/navigate_to_pose`
(en Milo todavía es `/navigate_to_pose`):

```bash
ros2 action send_goal /nav/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.572, y: -2.738}, orientation: {z: 0.1977, w: 0.9803}}}}" --feedback
```

o por nombre, sin hacer cuentas: `./sim.sh ir dining_table` (lee el `.yaml` y calcula $`z`$ y
$`w`$ solo).

<a id="origen"></a>

## 2. De dónde salen los 28 lugares

> **En palabras simples:** 6 los anotó a mano el equipo con Milo; los otros 22 los calcula el
> mismo script que arma la casa, que ya sabe dónde está cada mueble.

| Grupo | Lugares | De dónde salen |
| --- | --- | --- |
| Anotados con Milo (11-oct-2026) | `sofa`, `dining_table`, `table`, `base`, `indoor`, `outdoor` | `tf2_echo`, como arriba; son los del informe de Milo |
| Rulebook (18) | `laundry_table` (1), `washing_machine` (2), `shelf` (3), `laundry_trash_bin` (4), `bed` (5), `tv_stand` (6), `sofa_7` (7), `coffee_table` (8), `cabinet` (9), `refrigerator` (10), `counter` (11), `sink` (12), `cooking_table` (13), `dishwasher` (14), `kitchen_trash_bin` (15), `dinner_table` (16), `bedside_table` (17), `coat_rack` (18) | `LOCATIONS` de `andesrobot_gazebo/tools/gen_casa_robocup.py`: a `APPROACH` = 0.80 m de la cara del mueble, mirándolo. El número es el del plano del rulebook (los círculos azules de `robocup_casa.png`) |
| Centro de sala (4) | `laundry`, `bedroom`, `living_room`, `kitchen` | `ROOMS` del mismo script |

El generador calcula esas poses pero no las guarda (en Milo solo las usa para verificar la casa);
se copiaron a `casa_lugares.yaml` con el número del rulebook como comentario al final de la
línea. La del sofá se llama `sofa_7` porque `sofa` ya es la anotada con Milo. Las entradas
`entrada` y `salida` de `ROOMS` no se agregaron porque quedan afuera del mapa.

**Dos ajustes para Martha:**

- `outdoor` se corrió de x = −5.998 a **−5.700**: en la pose de Milo la huella de Martha tocaba el
  muro oeste.
- En `maps/casa.pgm` se limpiaron 21 celdas ocupadas afuera del muro oeste (x < −6.35). En la
  casa no hay nada ahí: eran restos del cuerpo de Milo en la partida, y dejaban a Martha encerrada
  al empezar (el planificador no encontraba ruta).

<a id="prueba"></a>

## 3. Qué revisa la prueba

`andesrobot_navigation/test/test_ubicaciones.py` (corre con `./sim.sh pruebas`) toma cada lugar
de `maps/casa_lugares.yaml`, pone la huella de Martha en su pose (cada 2 cm) y revisa que no toque
ninguna celda ocupada de `maps/casa.pgm`. Si un lugar falla, Nav2 nunca va a poder dejar a Martha
ahí, porque el controlador revisa la huella completa. También prueba cómo se buscan los nombres:
el nombre exacto o un prefijo que no se repita. Con los 28 lugares, `./sim.sh ir din` es ambiguo
(`dining_table` y `dinner_table`) y avisa; `./sim.sh ir dinn` va a `dinner_table`.

<a id="uso"></a>

## 4. Cómo usarlo

El paso a paso está en el [README principal](../../README.md#2-simulación). En corto:

| Paso | Comando |
| --- | --- |
| Navegación completa | `./sim.sh nav` |
| Ver los lugares | `./sim.sh ir --lista` |
| Ir a un lugar | `./sim.sh ir sofa` |
| Varios seguidos | `./sim.sh ir sofa base dining_table` |
| Anotar lugares | `./sim.sh shell` → `ros2 run tf2_ros tf2_echo map base_footprint` |
| Revisar que Martha quepa | `./sim.sh pruebas` |
| Regenerar la figura | `python3 ros2_ws/src/andesrobot_navigation/scripts/figuras_nav.py` |

También sirve **2D Goal Pose** en RViz (clic en el mapa y arrastrar para la dirección). Si tocas
el teleop le quitas el control a Nav2 mientras aprietes teclas.

**Archivos que más se tocan:**

| Archivo | Qué tiene |
| --- | --- |
| `maps/casa_lugares.yaml` | los lugares con nombre |
| `maps/casa.pgm` + `maps/casa.yaml` | el mapa de la casa |
| `andesrobot_navigation/config/nav2_params.yaml` | AMCL, costmaps, planificador, controlador (tolerancia de llegada 0.12 m y 0.15 rad) |
| `andesrobot_gazebo/tools/gen_casa_robocup.py` | la casa (copiado de milo_ws) |

<a id="pendientes"></a>

## 5. Pendientes

Según la [arquitectura](https://alejapinzonf.github.io/Robot_arquitecture_ROS2/), cada robot
tiene su propio mapa y su propia zona base:

- **Mapa propio de Martha.** El de `maps/` lo hizo Milo; falta mapear la casa con Martha.
- **`base_zone` de Martha.** Hoy `base` es la zona base anotada con Milo.
- **`/nav/set_initial_pose`.** Hoy AMCL parte en la pose que le da el launch (la partida de
  Martha); la arquitectura pide que el orquestador la mande con ese servicio.
