# Informe Persona C: Santiago Chacón

Práctica 5 (Nav2) y Proyecto Final (nodo de misión). Todo lo que aparece abajo se midió en la simulación real del paquete `dif_bot` (rama `persona-a` como base), con `use_ros2_control:=true`.

## Entorno de prueba

- ROS 2 Humble + Gazebo Fortress (ign gazebo 6.18) + Nav2 Humble + slam_toolbox, dentro de un contenedor Docker `ros:humble-ros-base`. La máquina de C trae ROS 2 Lyrical, que no tiene `ign_ros2_control` ni Nav2 instalado.
- Para que funcione en el contenedor sobre WSL2: `IGN_IP=127.0.0.1` (sin esto `ros_gz_sim create` nunca encuentra el servidor de Gazebo) y el socket X de WSLg montado con `DISPLAY=:0` y `LIBGL_ALWAYS_SOFTWARE=1`. El render Ogre v1 del mundo no soporta `--headless-rendering` (EGL) y abortaba con `Couldn't open X display`.
- Render por software, así que la CPU va saturada (load ~15, factor de tiempo real ~0.9). Esto explica varios de los ajustes de abajo.

## Archivos de C

| Archivo | Qué hace |
|---|---|
| `config/nav2_params.yaml` | Parámetros de Nav2 (se partió del `nav2_params.yaml` de Humble) + slam_toolbox |
| `config/waypoints.yaml` | Waypoints de la misión (frame `map`) |
| `launch/nav.launch.py` | Simulación + controladores + SLAM o AMCL + Nav2 + nodo de misión |
| `scripts/mission_node.py` | Nodo de misión con `nav2_simple_commander` y el servicio `/start_mission` |
| `scripts/cmd_vel_relay.py` | Reenvía `/cmd_vel` (salida de Nav2) a `/diff_drive_controller/cmd_vel_unstamped` |

Cómo correrlo:

```bash
ros2 launch dif_bot nav.launch.py slam:=true          # mapa en línea con slam_toolbox
ros2 launch dif_bot nav.launch.py slam:=false map:=<ruta>/map.yaml   # mapa guardado + AMCL
ros2 service call /start_mission std_srvs/srv/Trigger
ros2 topic echo /mission_status
```

`nav.launch.py` carga `joint_state_broadcaster` y `diff_drive_controller` con `spawner`, porque `sim.launch.py` no los carga.

## Práctica 5: Nav2

### 5.1 Nombres reales medidos en la simulación

Medido con `ros2 topic list`, `ros2 topic echo --once` y `tf2_tools view_frames`:

| Elemento | Valor real | Dónde se usa |
|---|---|---|
| Base frame | `base_link` | `robot_base_frame` de los costmaps y del bt_navigator, `base_frame_id` de AMCL, `base_frame` de slam_toolbox |
| Láser | `/scan`, frame `lidar_link`, 10 Hz (medido 9.9 Hz), rango 0.12–8.0 m | `observation_sources`, AMCL `laser_min/max_range` |
| Odometría | `/diff_drive_controller/odom` (frame `odom`, child `base_link`) | `odom_topic` del bt_navigator, controller_server y velocity_smoother |
| cmd_vel | Nav2 publica `/cmd_vel`; el controlador escucha `/diff_drive_controller/cmd_vel_unstamped` | `cmd_vel_relay.py` |
| Radio del robot | 0.26 m | `robot_radius` de los dos costmaps |

Árbol TF medido con ros2_control: `odom → base_link → {lidar_link, imu_link, camera_link, caster_link, left/right_wheel_link}`, y con SLAM `map → odom`.

**`base_footprint` queda desconectado.** El URDF declara `base_footprint → base_link` como joint fijo (robot_state_publisher lo publica en `/tf_static`), pero `diff_drive_controller` publica `odom → base_link`. Como un frame en TF solo puede tener un padre, gana el transform dinámico y `base_footprint` queda fuera del árbol. Por eso Nav2 usa `base_link`. Arreglarlo le corresponde a la Persona A (Daniela Molina): `base_frame_id: base_footprint` en `controllers.yaml`.

Radio: el chasis es una caja de 0.40 × 0.25 m centrada en `base_link`, con las ruedas en y = ±0.15 m y 0.03 m de ancho, o sea ±0.165 m. El robot gira sobre el eje de las ruedas (x = 0), así que el radio es la semidiagonal: √(0.20² + 0.165²) ≈ 0.26 m.

### 5.2 Ajustes respecto al `nav2_params.yaml` de Humble

1. `odom_topic: /diff_drive_controller/odom` también en `controller_server`. El YAML por defecto no lo trae y el controller_server de Humble se suscribe entonces a `/odom`, que con ros2_control no existe. El controlador veía siempre velocidad 0 y el robot se quedaba en `Failed to make progress`. Con este cambio la primera meta manual funcionó.
2. Costmap global con `rolling_window: true` (12 × 12 m) en modo SLAM, que `nav.launch.py` reescribe con `RewrittenYaml`. El mapa de slam_toolbox mide solo 4 × 4 m (80 × 80 celdas a 0.05 m) porque el láser solo golpea las cajas: las lecturas `inf` no marcan espacio libre. Con el costmap ajustado al mapa salía `The goal sent to the planner is off the global costmap`.
3. Controlador **Regulated Pure Pursuit** en lugar de DWB. Con la CPU saturada, DWB oscilaba y fallaba seguido con `Failed to make progress`; en la primera misión con DWB se alcanzaron 2 de 5 waypoints. RPP es más liviano y estable.
4. `controller_frequency: 10.0` (el lazo de 20 Hz no se cumplía) y `movement_time_allowance: 20.0` en el progress checker.
5. Rangos del costmap (`obstacle_max_range: 5.0`, `raytrace_max_range: 6.0`) y de AMCL (0.12–8.0 m), según el lidar.

### 5.3 Metas manuales

Con `ros2 launch dif_bot nav.launch.py slam:=true` se envió una meta a (1.5, 1.3) por la acción `/navigate_to_pose`, que es lo mismo que hace el botón "Nav2 Goal" de RViz: `ros2 action send_goal /navigate_to_pose ...`. El resultado fue `SUCCEEDED` en unos 40 s de tiempo de pared. Los intentos anteriores fallaron por los problemas 1 y 2 de la sección anterior.

### 5.4 ¿Qué planner se usa por defecto?

`planner_server` carga el plugin `GridBased` = **`nav2_navfn_planner/NavfnPlanner`** (NavFn), que es el valor por defecto de Nav2 Humble. Usa `use_astar: false`, es decir, Dijkstra sobre el costmap global, con `allow_unknown: true` (puede planificar por celdas desconocidas) y `tolerance: 0.5`. En los logs se ve como `GridBased: failed to create plan with tolerance 0.50` cuando la meta no es alcanzable.

### 5.5 ¿Qué comportamiento de recuperación se ejecuta?

El bt_navigator usa el árbol por defecto `navigate_to_pose_w_replanning_and_recovery.xml`:

- Si falla `ComputePathToPose`, limpia el costmap global y reintenta.
- Si falla `FollowPath`, limpia el costmap local y reintenta.
- Si sigue fallando, el `RecoveryNode` (hasta 6 reintentos) ejecuta en **RoundRobin**: limpiar los dos costmaps → **Spin** (1.57 rad) → **Wait** (5 s) → **BackUp** (0.30 m a 0.05 m/s).

Esto se vio en los logs del `behavior_server`: `Running backup … backup completed successfully`, `Running spin … Turning 1.57 for spin behavior … spin completed successfully` y `Running wait … wait completed successfully`, en ese orden rotativo. En la corrida final de la misión hubo un Spin, un Wait y un BackUp mientras el robot iba al primer waypoint, y después llegó igual.

## Proyecto Final: nodo de misión

### Diseño

- `mission_node` lee `waypoints.yaml` (`frame_id`, y por cada punto `name`, `x`, `y` y `yaw_deg`).
- El servicio **`/start_mission`** (`std_srvs/Trigger`) arranca la misión en un hilo aparte y responde enseguida (`Misión iniciada con 5 waypoints`). Si ya hay una misión en curso, responde `success: false`.
- Recorre los waypoints en orden con `BasicNavigator.goToPose()`. Cada uno tiene un tiempo máximo (`waypoint_timeout`, 120 s) y, si un waypoint falla, sigue con el siguiente.
- Publica el estado en **`/mission_status`** (`IDLE`, `WAITING_NAV2`, `GOING i/n`, `REACHED`/`FAILED`/`TIMEOUT` con el tiempo, y `FINISHED k/n`).
- Con `slam:=true` espera solo a `bt_navigator`, porque slam_toolbox en Humble no es un nodo lifecycle. Con `slam:=false` usa `waitUntilNav2Active(localizer='amcl')`.

Problemas que aparecieron y se corrigieron:

- `ValueError: generator already executing`: `BasicNavigator` usa el executor global de rclpy y el nodo de misión también estaba girando en él. Se le dio al nodo de misión su propio `SingleThreadedExecutor`.
- Poner `name='mission_node'` en el launch renombraba también el nodo interno de BasicNavigator, porque el remap `__node` afecta a todos los nodos del proceso. Se quitó.
- Los nodos Python (misión y relay) con `use_sim_time` recibían `/clock` a 1 kHz y gastaban ~50 % de CPU cada uno. No lo necesitan, así que se quitó.

### Waypoints

El mapa de SLAM solo cubre la zona donde el láser golpea las cajas: aproximadamente x de −2.5 a 1.5 m e y de −2 a 2 m. Las metas fuera de esa zona (por ejemplo (3.3, 0), detrás de la caja 1) fallaban en el planner. Los waypoints finales están dentro de esa zona y a 0.9 m o más de las esquinas de las cajas:

| # | Nombre | x | y | yaw |
|---|---|---|---|---|
| 1 | p1_noreste | 1.0 | 1.2 | 0° |
| 2 | p2_sureste | 0.8 | −1.2 | −90° |
| 3 | p3_suroeste | −1.0 | −0.2 | 180° |
| 4 | p4_noroeste | −1.2 | 1.2 | 90° |
| 5 | p5_inicio | 0.0 | 0.0 | 0° |

### Resultado

Corrida final con `ros2 launch dif_bot nav.launch.py slam:=true` + `ros2 service call /start_mission std_srvs/srv/Trigger`:

```
[estado] GOING 1/5 p1_noreste (1.00, 1.20)
[estado] REACHED p1_noreste en 67.8 s
[estado] GOING 2/5 p2_sureste (0.80, -1.20)
[estado] REACHED p2_sureste en 22.4 s
[estado] GOING 3/5 p3_suroeste (-1.00, -0.20)
[estado] REACHED p3_suroeste en 15.4 s
[estado] GOING 4/5 p4_noroeste (-1.20, 1.20)
[estado] REACHED p4_noroeste en 7.7 s
[estado] GOING 5/5 p5_inicio (0.00, 0.00)
[estado] REACHED p5_inicio en 9.1 s
[estado] FINISHED 5/5 waypoints alcanzados
```

Misión completa: **5/5 waypoints en 127 s** de tiempo de pared. Los tiempos son de pared, no de simulación, y el factor de tiempo real fue ~0.9.

Historial de intentos (todos con la misma simulación):

| Intento | Controlador | Waypoints | Resultado |
|---|---|---|---|
| 1 | DWB | originales (incluía (3.3, 0) detrás de la caja) | 2/5 |
| 2 | RPP | originales | 3/5 (fallan (3.3, 0) por estar fuera del mapa y (1.0, −1.5)) |
| 3 | RPP | dentro del mapa | **5/5** |

## Análisis y conclusiones

- La mayoría de los fallos no eran del algoritmo sino de configuración: topics de ros2_control distintos a los de Gazebo (`/diff_drive_controller/odom`, `cmd_vel_unstamped`), un `odom_topic` que faltaba en el controller_server, y el frame `base_footprint` suelto en TF. Conviene revisar `ros2 topic list` y `view_frames` antes de tocar ningún parámetro.
- El mundo `empty_world` tiene muy pocos obstáculos, así que el mapa de SLAM queda chico (4 × 4 m) y cualquier meta fuera de él no se puede planificar. Con un mundo con paredes (Warehouse o House, como decía el plan) el mapa cubriría toda la zona navegable y no haría falta el costmap global móvil.
- Sin GPU, el render por software se lleva casi toda la CPU y los lazos de Nav2 no cumplen su frecuencia. En esas condiciones RPP fue claramente más robusto que DWB.
- Pendiente con el equipo: probar `slam:=false` con el mapa y el `amcl.yaml` de la Persona B (Tomás Alvares), que todavía no están en el repo, y que la Persona A (Daniela Molina) decida si `diff_drive_controller` debe publicar `odom → base_footprint`.
