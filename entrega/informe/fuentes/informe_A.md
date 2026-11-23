# Informe Persona A

**Autora:** Daniela Molina (Prácticas 1 y 2: control/odometría y EKF)

> El robot `dif_bot` y el mundo fueron **generados por el equipo** (no son material de la cátedra). Se usan como base para las prácticas.
>
> **Estado del robot:** desde el merge con persona B, el robot es el de persona B (`urdf/robot.urdf.xacro`, con argumentos `use_ros2_control` y `use_ekf`) y el mundo por defecto es `rbtc_room`. **Las cifras de las secciones 1.x y 2.x se re-midieron el 2026-11-22 con el robot actual** (headless, una corrida por caso). Lo que aún menciona el robot anterior (con `base_footprint`, `lidar_link`, `camera_link` y mundo `empty_world`) está marcado como historial. Lo no medido figura como NO VERIFICADO.

## Práctica 1: Control y odometría

### 1.1 Joints, sensores y frames

Fuente: `src/dif_bot/urdf/robot.urdf.xacro`, `gz_control.xacro`, `ros2_control.xacro`.

**Joints**

Joints del URDF actual (robot de persona B). Eliminados con el cambio: `base_footprint_joint`, `caster_joint`, `lidar_joint`, `camera_joint`.

| Joint | Tipo | Padre → hijo | Origen (xyz) | Eje |
|---|---|---|---|---|
| `left_wheel_joint` | continuous | base_link → left_wheel | 0 0.19 -0.05 | 0 1 0 |
| `right_wheel_joint` | continuous | base_link → right_wheel | 0 -0.19 -0.05 | 0 1 0 |
| `front_joint` | fixed | base_link → front_caster | 0.16 0 -0.115 | — |
| `rear_joint` | fixed | base_link → rear_caster | -0.16 0 -0.115 | — |
| `laser_joint` | fixed | base_link → laser_frame | 0.1 0 0.13 | — |
| `imu_joint` | fixed | base_link → imu_link | 0 0 0 | — |

**Sensores (Gazebo Fortress)**

| Sensor | Tipo | Link (`reference`) | `ignition_frame_id` | Topic ROS | Frecuencia configurada (`update_rate`) |
|---|---|---|---|---|---|
| lidar | gpu_lidar | base_link (pose 0.1 0 0.13) | laser_frame | `/scan` (LaserScan) | 10 Hz |
| imu | imu | base_link | imu_link | `/imu` (Imu) | 50 Hz |

Los frames de sensores se fijan con `<ignition_frame_id>`. Medidos con el robot actual (re-medición del 2026-11-22, sin ros2_control; `scratchpad/p1m81/hz_h_*.txt`): `/scan` 10.00 Hz con frame `laser_frame`; `/imu` 49.87 Hz con frame `imu_link`. El robot actual no tiene cámara; las cifras de `/camera/image_raw` de la versión anterior se han quitado.

**Frames**

| Frame | Rol | Verificado |
|---|---|---|
| `odom` | Padre de la odometría | Sí, robot actual (re-medición 2026-11-22): `header.frame_id` de `/odom` (sin ros2_control) y de `/diff_drive_controller/odom` (con ros2_control) |
| `base_link` | Frame del robot; `child_frame_id` de `/odom` | Sí, robot actual (re-medición 2026-11-22): `child_frame_id` de `/odom` y de `/diff_drive_controller/odom`; `/tf` odom -> base_link en ambos modos |
| `left_wheel`, `right_wheel`, `front_caster`, `rear_caster` | Ruedas y ruedas locas | Sí, robot actual: `robot_state_publisher` registra los cuatro segmentos en el log de la simulación (sin ros2_control); `/tf_static` contiene `front_caster` y `rear_caster` (ambos modos) |
| `laser_frame`, `imu_link` | Sensores | Sí, robot actual: `/scan` con frame `laser_frame` e `/imu` con `imu_link` (ambos modos) |

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
- **Solución aplicada:** `<render_engine>ogre</render_engine>` (Ogre v1) en `worlds/empty_world.sdf` y, desde el merge, en `worlds/rbtc_room.sdf` (mundo por defecto). Con esto los sensores publican. Se mantiene `--headless-rendering` (EGL) en el modo `headless:=true`, necesario para arrancar el servidor sin ventana.
- Esto es un cambio de configuración, no un parche: Ogre v1 es el motor de render que soporta la cámara y el lidar en este entorno. No se ha probado en un equipo con GPU real.

**Resultado real** (launch completo, `headless:=true`, `use_ros2_control:=false`, robot actual; re-medición del 2026-11-22, dominio ROS 81)

> Hz con `ros2 topic hz` (7 s, reloj de pared); el resto con `ros2 topic info`, `ros2 node list`, `ros2 topic echo --once`, `tf2_echo` y `p1_motion.py` (pose real de Gazebo con `ign topic`). Salidas en `scratchpad/p1m81/`: `h_topics.txt`, `hz_h_*.txt`, `h_info_odom.txt`, `h_nodes.txt`, `h_tf_odom_base.txt`, `h_motion.txt`, `h_gyro.txt`. Esa carpeta no está versionada.

| Comando | Resultado |
|---|---|
| `ros2 topic list` | `/clock`, `/cmd_vel`, `/imu`, `/joint_states`, `/odom`, `/parameter_events`, `/robot_description`, `/rosout`, `/scan`, `/tf`, `/tf_static` |
| `ros2 topic hz /scan` | 9.997 Hz (desv. 0.00072 s) |
| `ros2 topic hz /imu` | 49.871 Hz (desv. 0.00051 s) |
| `ros2 topic hz /odom` | 29.412 Hz (desv. 0.00011 s) |
| `ros2 topic echo /scan --once` (`header.frame_id`) | `laser_frame` |
| `ros2 topic echo /imu --once` (`header.frame_id`) | `imu_link` |
| `ros2 topic echo /odom --once` (`header.frame_id`, `child_frame_id`) | `odom`, `base_link` |
| `ros2 topic info /odom -v` | 1 publisher, nodo `ros_gz_bridge`; 0 suscriptores |
| `ros2 node list` | `/robot_state_publisher`, `/ros_gz_bridge` |
| `/tf` (aristas en un mensaje) | solo `odom` → `base_link`, publicado por el plugin DiffDrive |
| `ros2 run tf2_ros tf2_echo odom base_link` | identidad al inicio (t ≈ 51 s de simulación) |
| prueba de movimiento: `/cmd_vel` a 0.2 m/s durante 5 s | `x` real de Gazebo 0.9856 m; `/odom` 0.9856 m (ver 1.5) |

Notas:
- `ros2 topic delay /odom` no dio una lectura interpretable en nuestra ejecución (el retardo medio salió como un valor absoluto de tiempo). **Queda sin medir.** Ver 1.5 para la alternativa medida.
- Los números de `pose.position.x` son una sola corrida; ver 1.5.
- La fila de `/camera/image_raw` ya no aplica: el sensor de cámara se eliminó con el robot de persona B.

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

**Resultado real** (launch completo, `headless:=true`, `use_ros2_control:=true`, robot actual; re-medición del 2026-11-22)

> Salidas en `scratchpad/p1m81/`: `rc_load_jsb.txt`, `rc_load_ddc.txt`, `rc_list.txt`, `rc_topics.txt`, `rc_nodes.txt`, `rc_hz_*.txt`, `rc_info_odom.txt`, `rc_odom_stamp.txt`, `rc_motion.txt`, `rc_gyro.txt`. Esa carpeta no está versionada.

- Ambos controladores cargan en estado **active** (verificado): `Successfully loaded controller joint_state_broadcaster into state active` y `Successfully loaded controller diff_drive_controller into state active`.
- Nodos (`ros2 node list`, verificado): `/controller_manager`, `/diff_drive_controller`, `/gz_ros2_control`, `/joint_state_broadcaster`, `/robot_state_publisher`, `/ros_gz_bridge`.
- Topics de sensores: `/scan` 9.99 Hz (frame `laser_frame`), `/imu` 47.98 Hz (frame `imu_link`), `/joint_states` 100.00 Hz. Hay un hueco máximo de 0.346 s en `/imu` durante la medida (desv. 0.019 s); no investigado.
- **La odometría NO está en `/odom`.** Está en `/diff_drive_controller/odom`: 49.99 Hz (desv. 0.00014 s), `Publisher count: 1` con nodo `diff_drive_controller`, `Subscription count: 0` (`ros2 topic info -v`). `header.frame_id` = `odom` y `child_frame_id` = `base_link` (lectura individual de `ros2 topic echo --once`, verificado). El comando NO va a `/cmd_vel`: el controlador escucha en `/diff_drive_controller/cmd_vel_unstamped`.
- `/tf`: un único `odom` → `base_link`, publicado por el controlador (`enable_odom_tf: true`). `/tf_static` contiene los segmentos de `robot_state_publisher`.
- Movimiento: 0.2 m/s durante 5 s en `/diff_drive_controller/cmd_vel_unstamped`: `x` real de Gazebo 0.9800 m y `/diff_drive_controller/odom` 0.9800 m (ver 1.5).
- El bridge de ros_gz en este modo **no** trae `/cmd_vel` ni `/odom` (solo `/clock`, `/scan` e `/imu`).

**Nota sobre el paquete `ign_ros2_control`:** la versión instalada (0.7.21) avisa de que el plugin está renombrado a `gz_ros2_control` (`gz_ros2_control/GazeboSimSystem` y `gz_ros2_control-system`). Se mantienen los nombres de la especificación; funcionan con aviso. Pendiente de decidir si migrar.

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Confirmar en pantalla que el robot se mueve igual en ambos modos con el mismo `cmd_vel` (en ros2_control, con el topic `/diff_drive_controller/cmd_vel_unstamped`).

### 1.4 ¿Qué controlador genera la odometría?

Borrador basado en el código y en lo medido:

- **Sin ros2_control** (`use_ros2_control:=false`): la odometría la genera el plugin nativo de Gazebo (Fortress) `ignition::gazebo::systems::DiffDrive` (definido en `urdf/gz_control.xacro`). Publica en el topic gz `odom` con frame `odom` y `child_frame_id` `base_link`, y el bridge lo expone como `/odom` (publisher: `ros_gz_bridge`). Lee `cmd_vel` de Gazebo.
- **Con ros2_control** (`use_ros2_control:=true`): la odometría la genera el controlador `diff_drive_controller` (DiffDriveController de ros2_control), cargado en el `controller_manager` que corre dentro de Gazebo mediante el plugin `ign_ros2_control`. Publica en `/diff_drive_controller/odom` según `config/controllers.yaml`.

Ambos calculan la odometría a partir de `wheel_separation` = 0.38 m y `wheel_radius` = 0.1 m (valores actuales en `gz_control.xacro` y `controllers*.yaml`, iguales al robot de persona B; antes 0.3 m y 0.05 m). El plugin DiffDrive publica `/odom` a `odom_publish_frequency` = 30 Hz (medido: 29.41 Hz). El controlador publica a `publish_rate` = 50 Hz (medido: 49.99 Hz).

Medido en la re-medición del 2026-11-22: en línea recta ambos dan la misma `x` que la pose real de Gazebo en cada corrida (0.9856 m sin ros2_control; 0.9800 m con ros2_control). En giro ambos subestiman el yaw real en torno al 5 % (ver 1.5). Con los datos de esta prueba, la diferencia visible entre ambos es la frecuencia y los nombres de topic.

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Comparar en pantalla la odometría de ambos modos tras la misma trayectoria y anotar la diferencia.

Comandos de medición (ejecutados en la re-medición; resultados abajo):

```bash
ros2 topic info /odom -v                         # sin ros2_control
ros2 topic info /diff_drive_controller/odom -v   # con ros2_control
ros2 node list
```

Publishers observados (medido con el robot actual, 2026-11-22): sin ros2_control, `ros2 topic info /odom -v` da `Publisher count: 1` con nodo `ros_gz_bridge` y `Subscription count: 0`; con ros2_control, `ros2 topic info /diff_drive_controller/odom -v` da `Publisher count: 1` con nodo `diff_drive_controller` y `Subscription count: 0` (ver 1.3). Salidas: `scratchpad/p1m81/h_info_odom.txt` y `rc_info_odom.txt`.

### 1.5 Estabilidad y latencia

Solo lo medido en la re-medición del 2026-11-22, robot actual, sin datos de la simulación en pantalla. Una sola sesión por modo (no hay repeticiones).

**Frecuencia (`ros2 topic hz`, 7 s, reloj de pared)** (re-medición 2026-11-22)

| Modo | Topic | Media | Desviación típica |
|---|---|---|---|
| sin ros2_control (headless) | `/odom` | 29.41 Hz | 0.00011 s |
| sin ros2_control (headless) | `/scan` | 10.00 Hz | 0.00072 s |
| sin ros2_control (headless) | `/imu` | 49.87 Hz | 0.00051 s |
| con ros2_control (headless) | `/diff_drive_controller/odom` | 49.99 Hz | 0.00014 s |
| con ros2_control (headless) | `/scan` | 9.99 Hz | 0.00080 s |
| con ros2_control (headless) | `/imu` | 47.98 Hz | 0.01922 s (hueco máximo 0.346 s) |
| con ros2_control (headless) | `/joint_states` | 100.00 Hz | 0.00012 s |

Salidas: `scratchpad/p1m81/hz_h_*.txt` (sin ros2_control) y `scratchpad/p1m81/rc_hz_*.txt` (con ros2_control). Las cifras de las corridas anteriores (`harness_*.out`, robot anterior) se han quitado de esta tabla.

**Movimiento y giro** (re-medición 2026-11-22, `scratchpad/p1m81/p1_motion.py`). Secuencia en tiempo de simulación: asentar 3 s, lineal 0.2 m/s durante 5 s, parada 1 s, giro 0.5 rad/s durante 5 s, parada 1 s. En cada parada se toma la pose real de Gazebo (`ign topic -e -t /world/rbtc_room/dynamic_pose/info`), que es la referencia, y se compara con la odometría y con la IMU. Cada modo usa su topic de comando (`/cmd_vel` sin ros2_control; `/diff_drive_controller/cmd_vel_unstamped` con ros2_control). Una corrida por modo.

| Modo | Fase | x real (Gazebo) | x odom | yaw real | yaw odom | yaw IMU |
|---|---|---|---|---|---|---|
| sin ros2_control | tras lineal | 0.9856 m | 0.9856 m | 0.0000 rad | 0.0000 rad | 0.0000 rad |
| sin ros2_control | tras giro | 0.9646 m (y 0.0218 m) | 0.9856 m (y 0) | 2.6138 rad | 2.4660 rad | 2.6138 rad |
| con ros2_control | tras lineal | 0.9800 m | 0.9800 m | 0.0000 rad | 0.0000 rad | 0.0000 rad |
| con ros2_control | tras giro | 0.9589 m (y 0.0222 m) | 0.9800 m (y 0) | 2.5892 rad | 2.4500 rad | 2.5892 rad |

Lectura:
- **El robot gira.** Tras el giro, el yaw real es 2.61 rad (sin ros2_control) y 2.59 rad (con ros2_control), frente a los 2.5 rad teóricos. Roll y pitch son 0.0000 en todo el ensayo y `z` queda en 0.1500 m: el chasis no apoya ni se inclina (la cara inferior de la caja queda a 7.5 cm del suelo). Verificado: se corrige el fallo del robot anterior.
- **La IMU coincide con la pose real de Gazebo** (diferencia < 1e-4 rad en ambos modos). La orientación de la IMU es absoluta (parte de yaw 0), pero `orientation_covariance[8]` sigue siendo 0.
- **La odometría subestima el giro real:** −5.7 % sin ros2_control y −5.4 % con ros2_control. En el tramo lineal, la odometría coincide con Gazebo a menos de 1 mm.
- **Confirmación independiente (giroscopio, `p1_gyro.py`, otra corrida del mismo giro):** la integral de `/imu` `angular_velocity.z` da 2.634 rad (sin ros2_control) y 2.626 rad (con ros2_control); la integral de `twist.angular.z` de la odometría da 2.499 y 2.465 rad. El giro real es mayor que el que predice la odometría de ruedas. Causa no determinada (posible deslizamiento; NO VERIFICADO).
- El tramo lineal da 0.9856 m y 0.9800 m frente a 1.00 m teóricos (1.4 % y 2 % menos). La causa no está determinada (NO VERIFICADO: posible arranque por aceleración).

Las corridas de `harness_*.out` del robot anterior (con tiempos de publicación y topics distintos) ya no se citan.

**Retardo.** `ros2 topic delay` no es interpretable en esta ejecución: devuelve valores absolutos del orden de 1.795e9 s, tanto para `/odom` como para `/diff_drive_controller/odom` (`scratchpad/p1m81/h_delay_odom.txt`, `rc_delay_odom.txt`). Como alternativa se midió la edad en tiempo de simulación (`now_sim − header.stamp` al recibir, `scratchpad/p1m81/p1_age.py`, 8 s):
- Sin ros2_control: `/odom` 0.00 ms (0 a 0 ms), `/imu` 0.00 ms, `/scan` 1.09 ms (1 a 3 ms).
- Con ros2_control: `/diff_drive_controller/odom` −0.01 ms (−1 a 0 ms), `/imu` 0.00 ms, `/scan` 1.19 ms (1 a 3 ms), `/joint_states` −0.02 ms (−2 a 1 ms).

Esto mide el desfase de los stamps respecto al reloj de simulación con resolución de 1 ms. **No es el retardo de transporte en tiempo real**: ese queda NO VERIFICADO. En la primera ejecución con ros2_control aparecieron algunos mensajes con stamp desfasado unos 270 s al arrancar; no se repitieron en la segunda y no está investigado.

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Medir con la simulación en pantalla y completar:

```bash
ros2 topic hz /odom                          # sin ros2_control: ____ Hz
ros2 topic hz /diff_drive_controller/odom    # con ros2_control: ____ Hz
ros2 topic delay /odom                       # retardo medio: ____ s
ros2 topic delay /diff_drive_controller/odom # retardo medio: ____ s
```

Valores de la re-medición del 2026-11-22 (headless, no en pantalla): `/odom` 29.41 Hz; `/diff_drive_controller/odom` 49.99 Hz; `ros2 topic delay` no interpretable (ver arriba).

### 1.6 ¿Cuál es más adecuado para un robot real?

Borrador: razonamiento sobre el código y sobre lo medido en simulación (no hay robot real):

- El plugin nativo de Gazebo (`DiffDrive`) es una herramienta de simulación: no tiene interfaz con hardware real ni parámetros de calidad de odometría configurables desde ROS.
- `ros2_control` separa el hardware (encoders y motores) del controlador (`diff_drive_controller`). El mismo controlador puede usarse en simulación y en el robot real cambiando solo la interfaz de hardware. Además expone `cmd_vel_timeout`, `open_loop` y parámetros de odometría.
- Por tanto, para un robot real, el enfoque con ros2_control parece el más adecuado. Coste: más configuración (plugin, `controller_manager`, carga manual de controladores), y los topics quedan bajo `/diff_drive_controller/`.
- Lo medido en simulación no separa los dos enfoques en precisión: ambos dan la misma odometría lineal y ambos subestiman el giro un 5 %. La diferencia medida es la frecuencia (30 Hz frente a 50 Hz) y el nombre de los topics. Esta conclusión sobre robot real es NO VERIFICADA.

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** Validar esta conclusión con sus resultados de 1.4 y 1.5 cuando tenga la simulación en pantalla.

## Práctica 2: EKF

Medidas de esta práctica con el robot actual (persona B, `urdf/robot.urdf.xacro`): mundo `rbtc_room`, `headless:=true`, Ogre v1, sin ruido de sensor en la simulación. Una corrida limpia por caso, 2026-11-22 (ver nota de 2.3). Las cifras del robot anterior se han retirado salvo donde se indica como historial. Lo que no se ha medido está marcado NO VERIFICADO.

### 2.1 Sensores fusionados

Fuente: `src/dif_bot/config/ekf.yaml`, modo 2D. Orden de los 15 campos de `*_config`: `[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`.

| Fuente | Topic | Campos fusionados | Por qué | Campos no fusionados y por qué |
|---|---|---|---|---|
| Odometría de rueda | `/odom` (gz DiffDrive) o `/diff_drive_controller/odom` (ros2_control) | `vx`, `vyaw` | La velocidad de las ruedas es la medida directa de avance y de giro. | `x`, `y`, `yaw` de la pose: son una integración de las ruedas y acumulan deriva; fusionarlos heredaría esa deriva. |
| IMU | `/imu` (`sensor_msgs/Imu`, frame `imu_link`, 50 Hz configurados; antes 100 Hz) | `vyaw` | Es la única medida de giro que aporta la IMU. | `yaw` absoluto: no lo aporta en esta simulación (ver hallazgo). `ax`: es ruidosa y `vx` ya viene de la odometría; usarla obligaría a tratar gravedad y sesgos. |

**Hallazgo histórico (robot anterior, corregido): la IMU funcionaba; el robot no giraba físicamente.** Se midió con la pose real de Gazebo en el robot anterior. Con el robot actual el giro sí es físico (ver 2.4).

- **Antes del cambio**, con `cmd_vel` angular 0.5 rad/s durante 5 s: `/odom` marcaba yaw 2.5000, pero la pose de `dif_bot` en Gazebo (`/world/empty_world/pose/info`) tenía yaw 0.0000 y `/imu` daba `orientation = (0,0,0,1)` y `angular_velocity.z = 0.0000`. Las ruedas sí giraban a ±1.5 rad/s (`/joint_states`, correcto: 0.5·0.15/0.05). Es decir, la rueda gira y el cuerpo no. La IMU dijo la verdad.
- **Causa raíz (robot anterior):** en `urdf/robot.urdf.xacro`, la caja de `base_link` (0.4 × 0.25 × 0.1, centrada en `base_link`) tenía su cara inferior a z = 0 del suelo, la misma altura que el fondo de las ruedas. El chasis apoyaba plano sobre el suelo con fricción (mu = 1 por defecto), y esa fricción frenaba el giro sobre sí mismo. En línea recta no se notaba: `x` era correcto (1.010 m reales frente a 1.0 de odometría) (robot anterior). Esa caja ya no existe: el robot actual tiene caja de 0.4 × 0.3 × 0.15 centrada en `base_link`, con cara inferior a z = -0.075 (por encima del fondo de las ruedas, z = -0.15). No se ha medido si el chasis toca el suelo con el robot actual: NO VERIFICADO. Lo que sí se midió es que el robot actual gira (2.4).
- **Cambio (robot anterior):** caja de `base_link` (visual y collision) con `<origin xyz="0 0 0.02">`, 2 cm de holgura. Ruedas, ejes, frames y nombres de topics sin cambios. Se probó también 5 cm: el robot quedó inclinado unos 0.25 rad al reposar, así que se descartó.
- **Después** (robot anterior, sin EKF, mismo ensayo): yaw real de gz 2.7753; `/imu` yaw 2.7753 (coincide con gz); `/imu` `angular_velocity.z = 0.5527` a mitad del giro y 0.0 al parar; `/odom` yaw 2.5000. La orientación del IMU ya es válida, pero `orientation_covariance[8]` sigue siendo 0 (gz no la publica).
- **Pendiente, no resuelto** (robot anterior): en reposo el robot queda con pitch de unos 0.1 rad, apoyado en la esquina delantera de la caja. El centro de masas, calculado con las masas del URDF, queda unos 1 mm por delante del eje de ruedas. Es un ajuste de masas o del caster y no se ha tocado.
- **Pendiente, no determinado:** con el robot actual `/odom` subestima el giro real en ~6% (2.5000 frente a 2.6623 rad, en las tres configuraciones de 2.4). La separación de ruedas de la URDF es 0.38 (y = ±0.19), igual que `wheel_separation` en `config/controllers*.yaml`, así que la diferencia no viene de ese parámetro. No se ha determinado la causa (posible deslizamiento). NO VERIFICADO.
- **Decisión sobre `imu0_config`:** se mantiene `vyaw` de la IMU. El yaw absoluto sigue sin fusionarse porque su covarianza es 0 y robot_localization la trataría como medida exacta. Esto se documenta en `config/ekf.yaml`.
- No se ha cambiado `process_noise_covariance` ni se ha añadido ruido de sensor. NO VERIFICADO: si un ruido de medida mayor en la IMU mejora algo.

### 2.2 Frame que publica el EKF

- El EKF publica `odom -> base_link` (`world_frame: odom`, `publish_tf: true`). Verificado con `view_frames` (`base_link` con padre `odom`, 30.2 Hz) y con el conteo de aristas de 2.3 (robot actual, 2026-11-22). No se repitió `ros2 topic echo --once /odom_filtered` en esta tanda: `header.frame_id` y `child_frame_id` de `/odom_filtered` quedan NO VERIFICADOS en esta tanda.
- `map -> odom` **no** lo publica el EKF. Lo publica un `static_transform_publisher` identidad lanzado por `ekf.launch.py` (argumento `launch_static_map_odom`, por defecto `true`). Es **provisional**: cuando SLAM o AMCL publiquen `map -> odom`, lanzar con `launch_static_map_odom:=false` para no tener dos publicadores de `map -> odom`.
- Con `use_ekf:=true` la odometría de rueda no publica `odom -> base_link`:
  - gz DiffDrive: `tf_topic` apunta a `/tf_ekf_disabled` (no se bridgea), y `/tf` no se bridgea. Sin `use_ekf` no cambia nada.
  - ros2_control: se carga `config/controllers_ekf.yaml` (copia de `controllers.yaml` con `enable_odom_tf: false`). Verificado con `ros2 param get /diff_drive_controller enable_odom_tf` (robot actual, 2026-11-22, caso C): `Boolean value is: False`.
- Con `use_ekf:=true` solo cambia el `tf_topic` del plugin DiffDrive en `gz_control.xacro` (`/tf_ekf_disabled`), y en ros2_control el yaml del controlador (`controllers_ekf.yaml`). Verificado en ejecución (casos A, B y C, ver 2.3): con `use_ekf:=true` la única arista `odom -> base_link` es la del EKF.

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

Resultados medidos (robot actual, 2026-11-22):

- `view_frames` (caso A, gz con IMU): árbol `map -> odom -> base_link`, con `base_link -> left_wheel` y `right_wheel` (20.2 Hz), casters y sensores (`imu_link`, `laser_frame`). `odom -> base_link` a 30.2 Hz. Copia del PDF dentro del repo: `src/dif_bot/docs/evidencias/frames_ekf.pdf` (16 KB). Para regenerarlo: `ros2 run tf2_tools view_frames` en el directorio de trabajo.
- `tf2_echo map base_link` (caso A, tras el movimiento, t ≈ 66.6 s de simulación): traslación (1.002, −0.000, 0.000), yaw 2.615 rad. Coincide con `/odom_filtered` en ese instante, porque `map -> odom` es identidad. La medida previa de identidad (antes del movimiento) de la versión anterior no se ha repetido: NO VERIFICADO.
- `ros2 topic info /tf -v` (caso A, gz con IMU): 2 publicadores, `ekf_filter_node` y `robot_state_publisher`. Caso C (ros2_control): 3 publicadores registrados, `ekf_filter_node`, `robot_state_publisher` y `diff_drive_controller` (este último con `enable_odom_tf: false`, ver 2.2).
- Aristas en `/tf`:
  - Caso A (gz, con IMU): 241 mensajes con `odom -> base_link` en 8 s (30.1 Hz). Caso B (sin IMU): 235 en 8 s (29.4 Hz).
  - Caso C (ros2_control): 241 mensajes con `odom -> base_link` en 8 s (30.1 Hz), con `diff_drive_controller` registrado en `/tf` pero con `enable_odom_tf: false`. Si el controlador también publicara esa arista, la cuenta sería mayor (~80 Hz).
- `tf2_monitor` no concluyó: reportó `<no authority available>` para `odom` y `base_link`. No lo uso como prueba.
- `view_frames` muestra `Broadcaster: default_authority`, sin nombre de nodo. Para saber quién publica, usar `topic info` y el conteo de aristas.

> Nota de entorno: en la primera tanda, procesos huérfanos de corridas anteriores (`parameter_bridge` y `static_transform_publisher`) publicaban `/clock` a la vez. Hubo 7365 retrocesos de `/clock` en 15 s, y el EKF y `robot_state_publisher` avisaban "jump back in time" (86 veces en el EKF) (medido con el robot anterior; re-medir). Tras matarlos, `/clock` no tuvo retrocesos. En la tanda actual (casos A, B y C) no aparece ningún "jump back in time" en `sim.log` ni en `ekf.log`. Las cifras de este informe son solo de corridas limpias.

### 2.4 Comparación /odom vs /odom_filtered

Comandos para la usuaria:

```bash
# Velocidad lineal de la odometría y del EKF (rqt_plot)
rqt_plot /odom/twist/twist/linear/x /odom_filtered/twist/twist/linear/x
# PlotJuggler (requiere GUI; en WSL, WSLg): Streaming > ROS2 Topic Subscriber, topics /odom y /odom_filtered
ros2 run plotjuggler plotjuggler
```

NO VERIFICADO: no se han abierto `rqt_plot` ni PlotJuggler (son GUI y no se ha hecho captura en esta sesión).

**Prueba de movimiento** (robot actual, 2026-11-22, una corrida por caso). Secuencia: lineal 0.2 m/s durante 5 s, parada 1 s, giro 0.5 rad/s durante 5 s, parada 1 s. La medida se toma al final de cada fase, con reloj de simulación. El yaw se calcula desde el cuaternión: `yaw = atan2(2(w·z + x·y), 1 − 2(y² + z²))`. La referencia es la pose de Gazebo (`/world/rbtc_room/pose/info`), leída antes y después del movimiento: `gz` antes x = 0.0000 m, yaw 0.0000; después, caso A: x = 0.9814 m, y = 0.0196 m, yaw 2.6623 rad; casos B y C: x = 0.9812 m, y = 0.0196 m, yaw 2.6621 rad. Con ros2_control (caso C) el comando va a `/diff_drive_controller/cmd_vel_unstamped`, no a `cmd_vel` (con `use_stamped_vel: false` el controlador no escucha `/diff_drive_controller/cmd_vel`).

| Caso | Topic | `yaw` a mitad de giro | `yaw` final | `x`, `y` final |
|---|---|---|---|---|
| Referencia gz (A) | — | — | 2.6623 | 0.9814, 0.0196 |
| A: EKF con IMU (`vyaw` de IMU) | `/odom` | 1.2340 | 2.5000 | 1.0002, 0.0000 |
| A: EKF con IMU | `/odom_filtered` | 1.3308 | 2.6149 | 1.0019, −0.0002 |
| A: EKF con IMU | `/imu` (yaw) | 1.3915 | 2.6623 | — |
| Referencia gz (B y C) | — | — | 2.6621 | 0.9812, 0.0196 |
| B: EKF sin IMU | `/odom` | 1.2370 | 2.5000 | 1.0000, 0.0000 |
| B: EKF sin IMU | `/odom_filtered` | 1.2240 | 2.4990 | 1.0013, −0.0004 |
| B: `/imu` (no fusionada) | `/imu` (yaw) | 1.3856 | 2.6621 | — |
| C: ros2_control + EKF con IMU | `/diff_drive_controller/odom` | 1.2400 | 2.5000 | 1.0000, −0.0000 |
| C: ros2_control + EKF con IMU | `/odom_filtered` | 1.2907 | 2.5813 | 1.0001, −0.0001 |
| C: ros2_control + EKF con IMU | `/imu` (yaw) | 1.3752 | 2.6621 | — |

Lectura:

- **Error de yaw frente a la referencia de gz** (referencia 2.6622 rad, media de A, B y C): odometría 0.162 rad (−6.1%) en las tres configuraciones; EKF con IMU 0.047 rad (−1.8%) en A y 0.081 rad (−3.0%) en C; EKF sin IMU 0.163 rad (−6.1%), igual que la odometría.
- **Con la IMU, el EKF reduce el error de giro** frente a la odometría: de 0.162 a 0.047 rad en A y a 0.081 rad en C. La IMU de la simulación no tiene ruido y su orientación coincide con la pose de gz (2.6623 frente a 2.6623), así que esta mejora es la de un sensor ideal; con una IMU real habría que medirla de nuevo. NO VERIFICADO: no se sabe por qué C mejora menos que A (no investigado).
- **Sin IMU, el EKF reproduce la odometría** (B: 2.4990 frente a 2.5000 rad; diferencias de 1.3e-3 m en x y 4e-4 m en y), que es lo esperable sin ruido: no hay nada que corregir.
- Frecuencias (`ros2 topic hz`, tiempo de pared; P1 simulaba en el mismo equipo): `/odom` 29.4 Hz (A y B); `/diff_drive_controller/odom` 50.0 Hz (C); `/odom_filtered` 30.0 Hz en A y C y 29.4 Hz en B; `/imu` 50.0 Hz. Con el equipo cargado, la tasa de pared puede quedar por debajo de la de simulación: NO VERIFICADO en tiempo de simulación.
- La pose física del robot se lee con `ign topic -e -t /world/rbtc_room/pose/info` (con `IGN_PARTITION=p2m82`). En las tres corridas el yaw de gz coincide con el yaw de `/imu`.

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

Con el mismo movimiento de 2.4 y con el robot actual (2026-11-22), el yaw final de `/odom_filtered` es 2.6149 rad con IMU (caso A) y 2.4990 rad sin IMU (caso B; `/odom` 2.5000). Frente a la referencia de gz (2.6622 rad), la IMU reduce el error de 0.163 a 0.047 rad. Las cifras del robot anterior (referencia 2.7753 rad) quedan retiradas. No se ha probado con ruido de sensor.

> **[OBSERVACIÓN PENDIENTE DE DANIELA]** En pantalla (Gazebo con GUI, sin `headless`), repetir el giro de 5 s a 0.5 rad/s con `use_imu:=true` y con `use_imu:=false` y anotar:
> - yaw final de `/odom`: ______
> - yaw final de `/odom_filtered` con IMU: ______
> - yaw final de `/odom_filtered` sin IMU: ______
> - ¿El robot gira visualmente unos 2.5 rad (unas 0.4 vueltas completas)? ______
