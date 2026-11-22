# Informe Persona A

> El robot `dif_bot` y el mundo `empty_world` fueron **generados por el equipo** (no son material de la cátedra). Se usan como base para las prácticas.

## Práctica 1: Control y odometría

### 1.1 Joints, sensores y frames

Fuente: `src/dif_bot/urdf/robot.urdf.xacro`, `gz_control.xacro`, `ros2_control.xacro`.

**Joints**

| Joint | Tipo | Padre → hijo | Origen (xyz) | Eje |
|---|---|---|---|---|
| `base_footprint_joint` | fixed | base_footprint → base_link | 0 0 0.05 | — |
| `left_wheel_joint` | continuous | base_link → left_wheel_link | 0 0.15 0 | 0 1 0 |
| `right_wheel_joint` | continuous | base_link → right_wheel_link | 0 -0.15 0 | 0 1 0 |
| `caster_joint` | fixed | base_link → caster_link | -0.15 0 -0.025 | — |
| `lidar_joint` | fixed | base_link → lidar_link | 0.1 0 0.07 | — |
| `imu_joint` | fixed | base_link → imu_link | 0 0 0 | — |
| `camera_joint` | fixed | base_link → camera_link | 0.19 0 0.03 | — |

**Sensores (Gazebo Fortress)**

| Sensor | Tipo | Link | Topic ROS | Frecuencia medida | `header.frame_id` medido |
|---|---|---|---|---|---|
| gpu_lidar | gpu_lidar | lidar_link | `/scan` (LaserScan) | 10 Hz | `lidar_link` |
| imu | imu | imu_link | `/imu` (Imu) | 100 Hz | `imu_link` |
| camera | camera | camera_link | `/camera/image_raw` (Image) | 10 Hz | `camera_link` |

Los frames de sensores se fijan con `<ignition_frame_id>` en el URDF (añadido en esta corrección). Los valores de la tabla de sensores (Hz y `header.frame_id`) son de terminal, salida no archivada.

**Frames**

| Frame | Rol | Verificado |
|---|---|---|
| `odom` | Padre de la odometría | Sí: `header.frame_id` de `/odom` (modo sin ros2_control; `harness_false2.out`) |
| `base_footprint` | Proyección en el suelo (link vacío) | Árbol de `robot_state_publisher` |
| `base_link` | Frame del robot; `child_frame_id` de `/odom` | Sí (`/odom` y `/tf`; `harness_false2.out`) |
| `left_wheel_link`, `right_wheel_link`, `caster_link` | Ruedas y rueda loca | Árbol de `robot_state_publisher` |
| `lidar_link`, `imu_link`, `camera_link` | Sensores | Sí (`header.frame_id` de cada topic; terminal, no archivado) |

### 1.2 Lanzamiento sin ros2_control

**Comandos**

```bash
source /opt/ros/humble/setup.bash
cd ~/Electronica/Robotica
colcon build --symlink-install
source install/setup.bash
ros2 launch dif_bot sim.launch.py use_ros2_control:=false headless:=true
```

**Problema encontrado y solución**

- Con el render `ogre2` del plugin Sensors del mundo, Gazebo terminaba al spawnear el robot: `terminate called ... Ogre::UnimplementedException ... GL3PlusTextureGpu::copyTo`. Reproducido con el launch real. Quitando solo la cámara, o solo el lidar, sigue crasheando; solo sin sensores carga (`diag_no_camera.log`, `diag_no_lidar.log` y `diag2_no_sensors.log` en el scratchpad). Es el backend GL3+ de Ogre2 en este WSL sin GPU real.
- **Solución aplicada:** `<render_engine>ogre</render_engine>` (Ogre v1) en `worlds/empty_world.sdf`. Con esto los sensores publican. Se mantiene `--headless-rendering` (EGL) en el modo `headless:=true`, necesario para arrancar el servidor sin ventana.
- Esto es un cambio de configuración, no un parche: Ogre v1 es el motor de render que soporta la cámara y el lidar en este entorno. No se ha probado en un equipo con GPU real.

**Resultado real** (launch completo, `headless:=true`, `use_ros2_control:=false`)

> Todas las cifras de esta sección (hz, frames, `/tf`, prueba de movimiento) son de terminal; la salida no está archivada en el scratchpad. Los `cs_*.log` son solo el stdout del launch y no contienen estas mediciones.

| Comando | Resultado |
|---|---|
| `ros2 topic hz /scan` | 10.02 Hz |
| `ros2 topic hz /imu` | 100.01 Hz |
| `ros2 topic hz /camera/image_raw` | 10.00 Hz |
| `ros2 topic hz /odom` | 49.99 Hz |
| `ros2 topic echo /scan --once --field header.frame_id` | `lidar_link` |
| `ros2 topic echo /imu --once --field header.frame_id` | `imu_link` |
| `ros2 topic echo /camera/image_raw --once --field header.frame_id` | `camera_link` |
| `ros2 topic echo /odom --once` (`header.frame_id`, `child_frame_id`) | `odom`, `base_link` |
| `/tf` | `odom` → `base_link` |
| prueba de movimiento: `/cmd_vel` a 0.2 m/s durante 5 s | `pose.position.x` pasa de ≈ 0 a 1.03 m |

Notas:
- `ros2 topic delay /odom` no dio una lectura interpretable en nuestra ejecución (el retardo medio salió como un valor absoluto de tiempo). **Queda sin medir.**
- Los números de `pose.position.x` son una sola corrida; ver 1.5.

### 1.3 Lanzamiento con ros2_control

**Comandos**

```bash
ros2 launch dif_bot sim.launch.py use_ros2_control:=true headless:=true
# en otra terminal, con el launch ya en marcha:
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active diff_drive_controller
ros2 topic list
ros2 topic hz /diff_drive_controller/odom
ros2 topic info /diff_drive_controller/odom -v
ros2 node list
```

**Resultado real** (launch completo, `headless:=true`, `use_ros2_control:=true`)

> Todas las cifras de este bloque (estados `active`, nodos, hz, frames, x = 0.94 m) son de terminal; la salida no está archivada en el scratchpad.

- Ambos controladores cargan en estado **active**: `Successfully loaded controller joint_state_broadcaster into state active` y `... diff_drive_controller into state active`.
- Nodos: `/controller_manager`, `/diff_drive_controller`, `/gz_ros2_control`, `/joint_state_broadcaster`, `/robot_state_publisher`, `/ros_gz_bridge`.
- Topics de sensores: `/scan` 10.00 Hz, `/imu` 99.99 Hz, `/camera/image_raw` 9.97 Hz, con frames `lidar_link`, `imu_link`, `camera_link`.
- **La odometría NO está en `/odom`.** Está en `/diff_drive_controller/odom` (49.99 Hz). El comando NO va a `/cmd_vel`: el controlador escucha en `/diff_drive_controller/cmd_vel_unstamped`.
- `/tf`: `odom` → `base_link`, publicado por el controlador (`enable_odom_tf: true`).
- Movimiento: publicando 0.2 m/s en `/diff_drive_controller/cmd_vel_unstamped`, `pose.position.x` pasa de ≈ 0 a 0.94 m.
- **No verificado:** el `header.frame_id` de `/diff_drive_controller/odom` en una lectura individual (en las dos corridas el primer mensaje se perdió por un aviso de `ros2 topic echo`). El `odom`→`base_link` sí está verificado en `/tf`.
- El bridge de ros_gz en este modo **no** trae `/cmd_vel` ni `/odom` (solo `/clock`, `/scan`, `/imu`, `/camera/image_raw`).

**Nota sobre el paquete `ign_ros2_control`:** la versión instalada (0.7.21) avisa de que el plugin está renombrado a `gz_ros2_control` (`gz_ros2_control/GazeboSimSystem` y `gz_ros2_control-system`). Se mantienen los nombres de la especificación; funcionan con aviso. Pendiente de decidir si migrar.

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Confirmar en pantalla que el robot se mueve igual en ambos modos con el mismo `cmd_vel` (en ros2_control, con el topic `/diff_drive_controller/cmd_vel_unstamped`).

### 1.4 ¿Qué controlador genera la odometría?

Borrador basado en el código y en lo medido:

- **Sin ros2_control** (`use_ros2_control:=false`): la odometría la genera el plugin nativo de Gazebo `gz::sim::systems::DiffDrive` (definido en `urdf/gz_control.xacro`). Publica en el topic gz `odom` con frame `odom` y `child_frame_id` `base_link`, y el bridge lo expone como `/odom` (publisher: `ros_gz_bridge`). Lee `cmd_vel` de Gazebo.
- **Con ros2_control** (`use_ros2_control:=true`): la odometría la genera el controlador `diff_drive_controller` (DiffDriveController de ros2_control), cargado en el `controller_manager` que corre dentro de Gazebo mediante el plugin `ign_ros2_control`. Publica en `/diff_drive_controller/odom` según `config/controllers.yaml`.

Ambos calculan la odometría a partir de `wheel_separation` = 0.3 m y `wheel_radius` = 0.05 m.

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Comparar en pantalla la odometría de ambos modos tras la misma trayectoria y anotar la diferencia.

Comandos de medición (completar):

```bash
ros2 topic info /odom -v                         # sin ros2_control
ros2 topic info /diff_drive_controller/odom -v   # con ros2_control
ros2 node list
```

Publishers observados: sin ros2_control, `ros_gz_bridge` con `Publisher count: 1` en `/odom` (archivado en `harness_false.out` y `harness_false2.out`, modo sin sensores; en el launch real no se guardó `ros2 topic info`: NO VERIFICADO allí); con ros2_control, `/diff_drive_controller` (ver 1.3).

### 1.5 Estabilidad y latencia

Solo lo medido (sin datos de la simulación en pantalla):

**Frecuencia (`ros2 topic hz`)**

| Corrida | Topic | Media | Desviación típica (rango) |
|---|---|---|---|
| sin ros2_control (launch real) | `/odom` | 49.99 Hz (terminal, no archivado) | — |
| sin ros2_control (harness sin sensores) | `/odom` | 49.95–50.00 Hz | 0.00004–0.00023 s |
| con ros2_control (harness sin sensores) | `/diff_drive_controller/odom` | 49.90–50.00 Hz | 0.00008–0.00030 s |
| con ros2_control (launch real) | `/diff_drive_controller/odom` | 49.99 Hz (terminal, no archivado) | — |

Fuentes de las desviaciones (salidas guardadas en el scratchpad del equipo): `harness_false.out` y `harness_false2.out` para el modo sin ros2_control; `harness_true.out` y `harness_true2.out` para el modo con ros2_control. Los launches reales no capturaron la desviación. `harness.sh` y los `.out` de estas pruebas no están versionados en el repo; están en el scratchpad de la sesión.

**Movimiento tras publicar `cmd_vel` a 0.2 m/s** (lectura de `pose.position.x`). Duración real del publish según el script que generó cada corrida: `harness_false.out` usó `timeout 4` (su etiqueta decía "3 s"); `harness_true.out` usó `timeout 5` (su etiqueta decía "4 s"); `harness_false2.out`, `harness_true2.out` y el launch real usaron `timeout 5`. Las etiquetas de los `.out` no coinciden con el timeout; los datos son los del timeout. La lectura posterior se hace inmediatamente después.

| Corrida | Topic de comando | x antes | x después |
|---|---|---|---|
| sin ros2_control, harness (`harness_false.out`) | `/cmd_vel` | no leído | 0.831 m |
| sin ros2_control, harness (`harness_false2.out`) | `/cmd_vel` | −1.8e-15 | 1.04 m |
| sin ros2_control, launch real | `/cmd_vel` | −1.8e-15 | 1.034 m (terminal, no archivado) |
| con ros2_control, harness (`harness_true.out`) | `/diff_drive_controller/cmd_vel_unstamped` | no leído | 0.92 m |
| con ros2_control, harness (`harness_true2.out`) | `/diff_drive_controller/cmd_vel_unstamped` | −1.8e-15 | 0.94 m |
| con ros2_control, launch real | `/diff_drive_controller/cmd_vel_unstamped` | −1.8e-15 | 0.94 m (terminal, no archivado) |

Lectura: con la misma velocidad comandada, el desplazamiento medido varía entre 0.83 y 1.04 m sin ros2_control, y entre 0.92 y 0.94 m con ros2_control. La diferencia de duración de publicación (4 s en `harness_false.out` frente a 5 s en las demás) explica en parte el rango sin ros2_control; el resto no tiene causa determinada. No se ha calculado un valor teórico para comparar.

**Harness vs launch real.** El harness (`harness.sh`, no versionado: está en el scratchpad de la sesión) bridgeaba `/cmd_vel` en los dos modos. El launch real **no** bridgea `/cmd_vel` con ros2_control: allí el comando es `/diff_drive_controller/cmd_vel_unstamped`. Los valores del harness con ros2_control se han medido sobre ese topic.

**Retardo** (`ros2 topic delay`): sin medir (ver 1.2).

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Medir con la simulación en pantalla y completar:

```bash
ros2 topic hz /odom                          # sin ros2_control: ____ Hz
ros2 topic hz /diff_drive_controller/odom    # con ros2_control: ____ Hz
ros2 topic delay /odom                       # retardo medio: ____ s
ros2 topic delay /diff_drive_controller/odom # retardo medio: ____ s
```

### 1.6 ¿Cuál es más adecuado para un robot real?

Razonamiento, **no medido**:

- El plugin nativo de Gazebo (`DiffDrive`) es una herramienta de simulación: no tiene interfaz con hardware real ni parámetros de calidad de odometría configurables desde ROS.
- `ros2_control` separa el hardware (encoders y motores) del controlador (`diff_drive_controller`). El mismo controlador puede usarse en simulación y en el robot real cambiando solo la interfaz de hardware. Además expone `cmd_vel_timeout`, `open_loop` y parámetros de odometría.
- Por tanto, para un robot real, el enfoque con ros2_control parece el más adecuado. Coste: más configuración (plugin, `controller_manager`, carga manual de controladores), y los topics quedan bajo `/diff_drive_controller/`.

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Validar esta conclusión con sus resultados de 1.4 y 1.5 cuando tenga la simulación en pantalla.

## Práctica 2: EKF

Medidas de esta práctica: mundo `empty_world`, `headless:=true`, Ogre v1, sin ruido en la simulación. Cada cifra viene de una corrida limpia (ver nota de 2.3). Lo que no se ha medido está marcado NO VERIFICADO.

### 2.1 Sensores fusionados

Fuente: `src/dif_bot/config/ekf.yaml`, modo 2D. Orden de los 15 campos de `*_config`: `[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`.

| Fuente | Topic | Campos fusionados | Por qué | Campos no fusionados y por qué |
|---|---|---|---|---|
| Odometría de rueda | `/odom` (gz DiffDrive) o `/diff_drive_controller/odom` (ros2_control) | `vx`, `vyaw` | La velocidad de las ruedas es la medida directa de avance y de giro. | `x`, `y`, `yaw` de la pose: son una integración de las ruedas y acumulan deriva; fusionarlos heredaría esa deriva. |
| IMU | `/imu` (`sensor_msgs/Imu`, frame `imu_link`, 100 Hz) | `vyaw` | Es la única medida de giro que aporta la IMU. | `yaw` absoluto: no lo aporta en esta simulación (ver hallazgo). `ax`: es ruidosa y `vx` ya viene de la odometría; usarla obligaría a tratar gravedad y sesgos. |

**Hallazgo: la IMU de esta simulación no aporta yaw ni giro válidos (medido).**

- `/imu` publica `orientation = (0, 0, 0, 1)` con `orientation_covariance[8] = 0` en todos los ensayos, también durante el giro.
- A mitad del giro (`/odom` a 0.5 rad/s) `angular_velocity.z = 0.0000` y `angular_velocity_covariance[8] = 0`. `linear_acceleration.z = 9.800`, así que la IMU mide la gravedad pero no el giro.
- Las covarianzas a 0 no se rechazan: robot_localization las fusiona como ruido de medida nulo. Ese es el problema práctico:
  - Con el yaw absoluto de la IMU fusionado, el yaw del EKF queda en 0: tras el giro de 2.5 rad, `/odom_filtered` marcó yaw 0.0000 (corrida con la primera configuración, `imu0_config` con yaw y vyaw).
  - Con solo `vyaw` de la IMU (0 con covarianza 0) junto al `vyaw` de la odometría (0.5), el EKF queda en la media: 0.25 rad/s a mitad del giro.
- **Decisión:** se quitó el yaw de la IMU de `imu0_config`. Se mantiene `vyaw` porque es la única medida de giro disponible, aunque en esta simulación empeora el resultado (ver 2.4 y 2.5).
- No se ha cambiado `process_noise_covariance` ni se ha añadido ruido de sensor al URDF. NO VERIFICADO: si un ruido de medida mayor en la IMU mejora algo.

### 2.2 Frame que publica el EKF

- El EKF publica `odom -> base_link` (`world_frame: odom`, `publish_tf: true`). Verificado con `ros2 topic echo --once /odom_filtered`: `header.frame_id: odom`, `child_frame_id: base_link`.
- `map -> odom` **no** lo publica el EKF. Lo publica un `static_transform_publisher` identidad lanzado por `ekf.launch.py` (argumento `launch_static_map_odom`, por defecto `true`). Es **provisional**: cuando SLAM o AMCL publiquen `map -> odom`, lanzar con `launch_static_map_odom:=false` para no tener dos publicadores de `map -> odom`.
- Con `use_ekf:=true` la odometría de rueda no publica `odom -> base_link`:
  - gz DiffDrive: `tf_topic` apunta a `/model/dif_bot/tf_ekf_disabled` (no se bridgea), y `/tf` no se bridgea. Sin `use_ekf` no cambia nada.
  - ros2_control: se carga `config/controllers_ekf.yaml` (copia de `controllers.yaml` con `enable_odom_tf: false`). Verificado con `ros2 param get /diff_drive_controller enable_odom_tf`: `Boolean value is: False`.
- Sin `use_ekf` el URDF generado con `xacro` es idéntico al de HEAD (diff vacío en ambos modos). Con `use_ekf:=true` solo cambian esas dos líneas.

### 2.3 Verificación del árbol TF

Comandos (con el launch real, `headless:=true`, y el EKF en marcha):

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames          # escribe frames.pdf y frames.gv en el directorio actual
ros2 topic info /tf -v
ros2 run tf2_ros tf2_monitor odom base_link
```

Qué comprobar:

- El árbol es `map -> odom -> base_link`, con `base_link` padre de ruedas y sensores.
- `odom -> base_link` tiene **un solo publicador**: `ekf_filter_node`, a ~30 Hz.
- `robot_state_publisher` publica las ruedas y los links fijos, pero no `odom`.
- En modo ros2_control, `diff_drive_controller` aparece como publicador registrado en `/tf`. Eso no basta para decir que publica TF: con `enable_odom_tf: false` no envía `odom -> base_link`. Se comprueba con el parámetro (ver 2.2) y contando las aristas que llegan a `/tf`.

Resultados medidos:

- `view_frames` (gz, con IMU): árbol `map -> odom -> base_link`, con `base_link -> left_wheel_link` y `right_wheel_link` (20.2 Hz) y sensores. `odom -> base_link` a 30.2 Hz. El PDF está en `scratchpad/caso1e_ekf_imu/frames_2026-11-22_17.00.00.pdf` (fuera del repo).
- `tf2_echo map base_link` (antes del movimiento): identidad (traslación 0, cuaternión `0 0 0 1`). NO VERIFICADO tras el movimiento.
- `ros2 topic info /tf -v` (gz, con IMU): 2 publicadores, `ekf_filter_node` y `robot_state_publisher`.
- Aristas en `/tf`:
  - gz, con IMU: 94 mensajes con `odom -> base_link` en una ventana de 5 s. La cifra es baja porque la ventana incluye el arranque del nodo; no es la tasa del EKF.
  - ros2_control, sonda dedicada (8 s): 241 mensajes con `odom -> base_link`, 30.1 Hz, con `diff_drive_controller` registrado en `/tf` pero con `enable_odom_tf: false`. Si el controlador también publicara esa arista, la cuenta sería mayor (~80 Hz).
- `tf2_monitor` no concluyó: reportó `<no authority available>` para `odom` y `base_link`. No lo uso como prueba.
- `view_frames` muestra `Broadcaster: default_authority`, sin nombre de nodo. Para saber quién publica, usar `topic info` y el conteo de aristas.

> Nota de entorno: en la primera tanda, procesos huérfanos de corridas anteriores (`parameter_bridge` y `static_transform_publisher`) publicaban `/clock` a la vez. Hubo 7365 retrocesos de `/clock` en 15 s, y el EKF y `robot_state_publisher` avisaban "jump back in time" (86 veces en el EKF). Tras matarlos, `/clock` no tuvo retrocesos. Las cifras de este informe son solo de corridas limpias.

### 2.4 Comparación /odom vs /odom_filtered

Comandos para la usuaria:

```bash
# Velocidad lineal de la odometría y del EKF (rqt_plot)
rqt_plot /odom/twist/twist/linear/x /odom_filtered/twist/twist/linear/x
# PlotJuggler (requiere GUI; en WSL, WSLg): Streaming > ROS2 Topic Subscriber, topics /odom y /odom_filtered
ros2 run plotjuggler plotjuggler
```

NO VERIFICADO: no se han abierto `rqt_plot` ni PlotJuggler (son GUI y no se ha hecho captura en esta sesión).

**Prueba de movimiento.** Secuencia: lineal 0.2 m/s durante 5 s, parada 1 s, giro 0.5 rad/s durante 5 s, parada 1 s. La medida se toma al final de cada fase, con reloj de simulación. El yaw se calcula desde el cuaternión: `yaw = atan2(2(w·z + x·y), 1 − 2(y² + z²))`. Tras la fase lineal, `/odom` y `/odom_filtered` dan `x = 1.0000 m` en las tres corridas.

| Corrida | Topic | `yaw` a mitad de giro | `yaw` final | `x`, `y` final |
|---|---|---|---|---|
| gz, con IMU (`vyaw` de IMU) | `/odom` | 1.2410 | 2.5000 | 1.0000, −0.0000 |
| | `/odom_filtered` | 0.3055 | 0.6250 | 1.0000, −0.0000 |
| gz, sin IMU | `/odom` | 1.2395 | 2.5000 | 1.0000, −0.0000 |
| | `/odom_filtered` | 1.2200 | 2.5000 | 1.0002, −0.0003 |
| ros2_control, con IMU | `/diff_drive_controller/odom` | 1.2450 | 2.5000 | 1.0000, −0.0000 |
| | `/odom_filtered` | 0.2988 | 0.6250 | 1.0000, −0.0000 |

Lectura honesta:

- **Sin IMU, el EKF reproduce la odometría** (diferencias de 2e-4 m en x y 3e-4 m en y; yaw final igual). Eso es lo esperable en simulación sin ruido: `/odom` ya es casi perfecta y el EKF no tiene nada que corregir.
- **Con la IMU de esta simulación, el EKF empeora el giro**: yaw final 0.625 rad frente a 2.5 rad de la odometría. La causa está medida en 2.1 (giro de la IMU a 0 con covarianza 0). **No se afirma que el EKF mejore nada en esta simulación.**
- Frecuencias: `/odom_filtered` 29.98–30.00 Hz en las cuatro corridas. `/diff_drive_controller/odom` 49.9 Hz en la sonda de ros2_control. `/odom` no se ha vuelto a medir en esta sesión (NO VERIFICADO aquí; ver 1.5).
- La pose física del robot en gz no se pudo leer (`gz topic` respondió "An instance of Gazebo is not running" en su intento), así que **no se sabe si el robot gira 2.5 rad de verdad**. NO VERIFICADO.

### 2.5 Efecto de desactivar la IMU

Pasos exactos:

```bash
# Terminal 1: simulación con EKF (sin ros2_control)
ros2 launch dif_bot sim.launch.py use_ros2_control:=false use_ekf:=true
# Terminal 2: EKF sin IMU (imu0_config queda todo False)
ros2 launch dif_bot ekf.launch.py use_imu:=false
# Terminal 3: comprobar y mover
ros2 topic hz /odom_filtered
```

Con el mismo movimiento de 2.4, el yaw final de `/odom_filtered` pasa de 0.6250 rad (con IMU) a 2.5000 rad (sin IMU), igual que `/odom`. Lo que se ve aquí es el efecto de la IMU concreta de esta simulación, no una propiedad del EKF. No se ha probado con ruido ni con una IMU que sí mida giro.

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** En pantalla (Gazebo con GUI, sin `headless`), repetir el giro de 5 s a 0.5 rad/s con `use_imu:=true` y con `use_imu:=false` y anotar:
> - yaw final de `/odom`: ______
> - yaw final de `/odom_filtered` con IMU: ______
> - yaw final de `/odom_filtered` sin IMU: ______
> - ¿El robot gira visualmente unos 2.5 rad (unas 0.4 vueltas completas)? ______
