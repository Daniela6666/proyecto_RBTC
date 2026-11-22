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

| Sensor | Tipo | Link | Topic ROS (bridge) | Frecuencia | Parámetros |
|---|---|---|---|---|---|
| gpu_lidar | gpu_lidar | lidar_link | `/scan` (LaserScan) | 10 Hz | 360 muestras, rango 0.12–8 m |
| imu | imu | imu_link | `/imu` (Imu) | 100 Hz | — |
| camera | camera | camera_link | `/camera/image_raw` (Image) | 10 Hz | 320x240 |

**Frames**

| Frame | Rol |
|---|---|
| `odom` | Padre de la odometría (`header.frame_id` de `/odom`). |
| `base_footprint` | Proyección en el suelo del robot (link vacío). |
| `base_link` | Frame del robot; `child_frame_id` de la odometría. Está a 0.05 m sobre base_footprint. |
| `left_wheel_link`, `right_wheel_link` | Ruedas (cilindro r=0.05). |
| `caster_link` | Rueda loca (esfera r=0.025, fricción 0). |
| `lidar_link`, `imu_link`, `camera_link` | Sensores. |

Verificado en ejecución (sin sensores, ver 1.2): `odom` → `base_link` en `/tf`. **El `frame_id` de los mensajes de `/scan`, `/imu` y `/camera/image_raw` NO VERIFICADO**: la simulación con sensores no arranca en este equipo (ver 1.2).

### 1.2 Lanzamiento sin ros2_control

**Comandos**

```bash
source /opt/ros/humble/setup.bash
cd ~/Electronica/Robotica
colcon build --symlink-install
source install/setup.bash
ros2 launch dif_bot sim.launch.py use_ros2_control:=false headless:=true
```

**Resultado real**

- `xacro urdf/robot.urdf.xacro use_ros2_control:=false`: sin error. `check_urdf`: "Successfully Parsed XML", 8 links.
- `colcon build --symlink-install`: OK (paquete `dif_bot`).
- Lanzamiento completo: **Gazebo termina con código 134** al spawnear el robot (`terminate called ... Ogre::UnimplementedException ... GL3PlusTextureGpu::copyTo`). El crash viene del render de Ogre2 que usan los sensores. Se reprodujo con cada sensor por separado (sin lidar y sin cámara también crashea), y **sin ningún sensor el robot se carga bien**. Es un problema del entorno (WSL sin GPU real), no del URDF. `--headless-rendering` (EGL) es necesario para que el servidor arranque sin pantalla.
- Por tanto, los topics de sensores (`/scan`, `/imu`, `/camera/image_raw`) **NO VERIFICADO: falla el render en WSL**.

**Verificación de la parte no visual** (mismo URDF sin los tres sensores, mismo bridge, gz en modo servidor; harness de diagnóstico en el equipo, no es el launch final):

```bash
ros2 topic list
ros2 topic hz /odom
ros2 topic info /odom -v
ros2 node list
ros2 topic delay /odom
```

| Comando | Resultado |
|---|---|
| `ros2 topic list` | `/clock /cmd_vel /joint_states /odom /tf /tf_static /robot_description ...` |
| `ros2 topic hz /odom` | **50 Hz** (media 49.99–50.00, min/max 0.020 s) |
| `ros2 topic info /odom -v` | 1 publisher: nodo `ros_gz_bridge`, tipo `nav_msgs/msg/Odometry` |
| `ros2 node list` | `/robot_state_publisher`, `/ros_gz_bridge` |
| frames de `/odom` | `header.frame_id` = `odom`, `child_frame_id` = `base_link` |
| `/tf` | `odom` → `base_link` |
| `/joint_states` | `['left_wheel_joint', 'right_wheel_joint']` |
| prueba de movimiento | `/cmd_vel` con x=0.2 m/s durante 4 s → `pose.position.x` ≈ 0.83 m |

Nota: `ros2 topic delay` no dio una lectura interpretable en nuestra ejecución (el promedio de retardo salió como un valor absoluto de tiempo). Queda para medir en pantalla.

### 1.3 Lanzamiento con ros2_control

**Comandos**

```bash
ros2 launch dif_bot sim.launch.py use_ros2_control:=true headless:=true
# en otra terminal:
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active diff_drive_controller
ros2 topic list
ros2 topic hz /diff_drive_controller/odom
ros2 topic info /diff_drive_controller/odom -v
ros2 node list
```

**Resultado real**

- `xacro ... use_ros2_control:=true` y `check_urdf`: sin error (bloque `<ros2_control name="IgnitionSystem">` y `<gazebo>` con `ign_ros2_control-system` presentes).
- El plugin `ign_ros2_control-system` **carga** (el `LD_LIBRARY_PATH` no basta; el launch añade `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` con las `lib` de `AMENT_PREFIX_PATH`). El paquete instalado avisa de que el nombre se ha renombrado a `gz_ros2_control` (`gz_ros2_control/GazeboSimSystem`, `gz_ros2_control-system`). Se mantienen los nombres de la especificación; hay que decidir si migrar.
- Lanzamiento completo: **mismo crash de Ogre2** que en 1.2, al spawnear el robot con sensores.
- Con el harness sin sensores: `joint_state_broadcaster` y `diff_drive_controller` cargan en estado **active**. Nodos: `/controller_manager`, `/diff_drive_controller`, `/gz_ros2_control`, `/joint_state_broadcaster`.
- **La odometría NO se publica en `/odom`**, sino en `/diff_drive_controller/odom` (50 Hz, 1 publisher). El `cmd_vel` del controlador se escucha en `/diff_drive_controller/cmd_vel_unstamped`, no en `/cmd_vel`. Un remapeo en el launch no es posible porque el controller_manager corre dentro del proceso de Gazebo. Para tener `/odom` y `/cmd_vel` hace falta un relay (`topic_tools`), que **no está instalado** en el equipo.
- Prueba de movimiento: publicando 0.2 m/s en `/diff_drive_controller/cmd_vel_unstamped` durante 4 s, `pose.position.x` pasa a ≈ 0.92 m.
- `/tf`: `odom` → `base_link` (publicado por el controlador, `enable_odom_tf: true`).

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Confirmar en pantalla (cuando el render funcione) que el robot se mueve igual en ambos modos con el mismo `cmd_vel`.

### 1.4 ¿Qué controlador genera la odometría?

Borrador basado en el código y en lo medido:

- **Sin ros2_control** (`use_ros2_control:=false`): la odometría la genera el plugin nativo de Gazebo `gz::sim::systems::DiffDrive` (definido en `urdf/gz_control.xacro`). Publica en el topic gz `odom` con frame `odom` y `child_frame_id` `base_link`, y el bridge lo expone como `/odom`. Frecuencia configurada: 50 Hz. Lee `cmd_vel` de Gazebo y publica `/tf`.
- **Con ros2_control** (`use_ros2_control:=true`): la odometría la genera el controlador `diff_drive_controller` (DiffDriveController de ros2_control), cargado en el `controller_manager` que corre dentro de Gazebo mediante `ign_ros2_control`. Publica en `/diff_drive_controller/odom` (50 Hz) según `config/controllers.yaml`.

Ambos calculan la odometría a partir de `wheel_separation` = 0.3 y `wheel_radius` = 0.05.

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Comparar en pantalla la odometría de ambos modos tras la misma trayectoria y anotar la diferencia.

Medición de referencia (completar):

```bash
ros2 topic info /odom -v                # sin ros2_control
ros2 topic info /diff_drive_controller/odom -v   # con ros2_control
ros2 node list
```

Publishers observados: sin ros2_control `ros_gz_bridge` (1); con ros2_control `/diff_drive_controller` (ver 1.3).

### 1.5 Estabilidad y latencia

Borrador (solo lo medido en este equipo, sin sensores):

- Frecuencia de `/odom` (sin ros2_control): 50 Hz estables (desviación estándar ≈ 0.00005 s).
- Frecuencia de `/diff_drive_controller/odom` (con ros2_control): 50 Hz estables (desviación estándar ≈ 0.00012 s).

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Medir con la simulación completa (con sensores) y completar:

```bash
ros2 topic hz /odom                       # sin ros2_control: ____ Hz
ros2 topic hz /diff_drive_controller/odom # con ros2_control: ____ Hz
ros2 topic delay /odom                    # retardo medio: ____ s (requiere header.stamp; Odometry lo tiene)
ros2 topic delay /diff_drive_controller/odom
```

### 1.6 ¿Cuál es más adecuado para un robot real?

Borrador (razonamiento, no medido):

- El plugin nativo de Gazebo (`DiffDrive`) es una herramienta de simulación: no tiene interfaz con hardware real ni parámetros de calidad de odometría configurables desde ROS.
- `ros2_control` separa el hardware (interfaz de encoders y motores) del controlador (`diff_drive_controller`). El mismo controlador sirve en simulación y en el robot real cambiando solo el hardware. Además expone `cmd_vel_timeout`, `open_loop` y covarianzas de odometría.
- Por eso, para un robot real, el enfoque con ros2_control es el más adecuado. Coste: más configuración (plugin, `controller_manager`, carga manual de controladores) y los topics quedan bajo `/diff_drive_controller/`.

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Validar esta conclusión con sus resultados de 1.4 y 1.5 cuando tenga la simulación con sensores funcionando.

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
