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

Los frames de sensores se fijan con `<ignition_frame_id>` en el URDF (añadido en esta corrección; el frame resultante se midió, ver tabla).

**Frames**

| Frame | Rol | Verificado |
|---|---|---|
| `odom` | Padre de la odometría | Sí: `header.frame_id` de `/odom` (modo sin ros2_control) |
| `base_footprint` | Proyección en el suelo (link vacío) | Árbol de `robot_state_publisher` |
| `base_link` | Frame del robot; `child_frame_id` de `/odom` | Sí (`/odom` y `/tf`) |
| `left_wheel_link`, `right_wheel_link`, `caster_link` | Ruedas y rueda loca | Árbol de `robot_state_publisher` |
| `lidar_link`, `imu_link`, `camera_link` | Sensores | Sí (`header.frame_id` de cada topic) |

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

- Con el render `ogre2` del plugin Sensors del mundo, Gazebo terminaba al spawnear el robot: `terminate called ... Ogre::UnimplementedException ... GL3PlusTextureGpu::copyTo`. Reproducido con el launch real. Sin ningún sensor el robot carga bien, y cada sensor por separado también crashea con ogre2. Es el backend GL3+ de Ogre2 en este WSL sin GPU real.
- **Solución aplicada:** `<render_engine>ogre</render_engine>` (Ogre v1) en `worlds/empty_world.sdf`. Con esto los sensores publican. Se mantiene `--headless-rendering` (EGL) en el modo `headless:=true`, necesario para arrancar el servidor sin ventana.
- Esto es un cambio de configuración, no un parche: Ogre v1 es el motor de render que soporta la cámara y el lidar en este entorno. No se ha probado en un equipo con GPU real.

**Resultado real** (launch completo, `headless:=true`, `use_ros2_control:=false`)

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

Publishers observados: sin ros2_control, `ros_gz_bridge` (1 publisher en `/odom`); con ros2_control, `/diff_drive_controller` (ver 1.3).

### 1.5 Estabilidad y latencia

Solo lo medido (sin datos de la simulación en pantalla):

**Frecuencia (`ros2 topic hz`)**

| Corrida | Topic | Media | Desviación típica (rango) |
|---|---|---|---|
| sin ros2_control (launch real) | `/odom` | 49.99 Hz | — |
| sin ros2_control (harness sin sensores) | `/odom` | 49.95–50.00 Hz | 0.00004–0.00023 s |
| con ros2_control (harness sin sensores) | `/diff_drive_controller/odom` | 49.90–50.00 Hz | 0.00008–0.00030 s |
| con ros2_control (launch real) | `/diff_drive_controller/odom` | 49.99 Hz | — |

Fuentes de las desviaciones (salidas guardadas en el scratchpad del equipo): `harness_false.out` y `harness_false2.out` para el modo sin ros2_control; `harness_true.out` y `harness_true2.out` para el modo con ros2_control. Los launches reales no capturaron la desviación.

**Movimiento tras publicar `cmd_vel` a 0.2 m/s** (lectura de `pose.position.x`; el publish dura `timeout 5`, es decir ≈ 5 s, y la lectura se hace después):

| Corrida | Topic de comando | x antes | x después |
|---|---|---|---|
| sin ros2_control, harness (`harness_false.out`) | `/cmd_vel` | no leído | 0.831 m |
| sin ros2_control, harness (`harness_false2.out`) | `/cmd_vel` | −1.8e-15 | 1.04 m |
| sin ros2_control, launch real | `/cmd_vel` | −1.8e-15 | 1.034 m |
| con ros2_control, harness (`harness_true.out`) | `/diff_drive_controller/cmd_vel_unstamped` | no leído | 0.92 m |
| con ros2_control, harness (`harness_true2.out`) | `/diff_drive_controller/cmd_vel_unstamped` | −1.8e-15 | 0.94 m |
| con ros2_control, launch real | `/diff_drive_controller/cmd_vel_unstamped` | −1.8e-15 | 0.94 m |

Lectura: con la misma velocidad comandada, el desplazamiento medido varía entre 0.83 y 1.04 m sin ros2_control, y entre 0.92 y 0.94 m con ros2_control. La causa de la variación no está determinada: puede venir del arranque y la aceleración del robot y del tiempo entre la publicación y la lectura. No se ha calculado un valor teórico para comparar.

**Harness vs launch real.** El harness (`/tmp/.../scratchpad/harness.sh`) bridgeaba `/cmd_vel` en los dos modos. El launch real **no** bridgea `/cmd_vel` con ros2_control: allí el comando es `/diff_drive_controller/cmd_vel_unstamped`. Los valores del harness con ros2_control se han medido sobre ese topic.

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
<!-- TODO -->

### 2.1 Sensores fusionados
<!-- TODO -->

### 2.2 Frame que publica el EKF
<!-- TODO -->

### 2.3 Verificación del árbol TF
<!-- TODO -->

### 2.4 Comparación /odom vs /odom_filtered
<!-- TODO -->

### 2.5 Efecto de desactivar la IMU
<!-- TODO -->
