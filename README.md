# Robótica Móvil: proyecto final

Simulación de un robot diferencial (`dif_bot`) en Gazebo Fortress con ROS 2 Humble, usando el puente `ros_gz`, en el mundo `rbtc_room` (8 x 8 m, tres obstáculos).

Incluye: control y odometría con alternativa `ros2_control` (Práctica 1, persona A), fusión sensorial con EKF (Práctica 2, persona A), SLAM y AMCL (Prácticas 3 y 4, persona B) y Nav2 con nodo de misión (persona C).

## Equipo y reparto

| Persona | Responsable | Prácticas |
|---|---|---|
| A | Daniela Molina | Prácticas 1 y 2: control/odometría y EKF |
| B | Tomas Alvarez Jaimes | SLAM y AMCL (Prácticas 3 y 4) |
| C | Santiago Chacon Guerrero | Nav2 y misión |

Ramas individuales: `persona-a`, `persona-b`, `persona-c`. Todo entra a `main` mediante PR.

## Versiones

- ROS 2 Humble
- Gazebo Fortress (Ignition)
- ros_gz

## Mundo

Mundo `rbtc_room` (`src/dif_bot/worlds/rbtc_room.sdf`), el mundo por defecto: suelo, luz `sun`, paredes de 8 x 8 m y tres cajas estáticas como obstáculos. Plugin de sensores con `render_engine` ogre (Ogre v1), ver nota de entorno más abajo.

`empty_world.sdf` se conserva solo como referencia de las medidas de la Práctica 1 y 2 (ver `src/dif_bot/docs/informe_A.md`); no es el mundo por defecto.

## Robot

Un único robot: `src/dif_bot/urdf/robot.urdf.xacro`, con la geometría, joints, sensores y nombres del robot de persona B:

- Ruedas motrices `left_wheel` / `right_wheel` (radio 0.1 m, separación 0.38 m), joints `left_wheel_joint` / `right_wheel_joint`.
- Casters `front_caster` / `rear_caster` (sin fricción).
- Frames `laser_frame` (LiDAR `/scan`) e `imu_link` (IMU `/imu`).

Argumentos del xacro (los pasa `sim.launch.py`):

- `use_ros2_control` (defecto `false`): `true` carga `ros2_control.xacro` (hardware `ign_ros2_control`) y `false` carga `gz_control.xacro` (DiffDrive nativo de Gazebo, odometría a 30 Hz).
- `use_ekf` (defecto `false`): `true` hace que el TF `odom -> base_link` lo publique el EKF y no la odometría de rueda.

Con `use_ros2_control:=false` el robot es funcionalmente el de persona B. El robot anterior de persona A (con cámara y `base_footprint`) se ha eliminado.

## Dependencias

```bash
sudo apt-get update && sudo apt-get install -y ros-humble-ign-ros2-control ros-humble-plotjuggler-ros ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-robot-localization ros-humble-xacro ros-humble-rqt-plot ros-humble-tf2-tools
sudo apt-get install -y ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-ros-gz ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard python3-colcon-common-extensions ros-humble-nav2-simple-commander python3-yaml python3-numpy python3-pil
```

## Estructura del repo

```
src/dif_bot/
├── urdf/       # robot.urdf.xacro, gz_control.xacro, ros2_control.xacro
├── worlds/     # rbtc_room.sdf (por defecto), empty_world.sdf (referencia)
├── rviz/       # dif_bot.rviz
├── config/     # controllers*.yaml, ekf.yaml, amcl.yaml, slam.yaml, nav2_params.yaml, waypoints.yaml, rbtc.rviz
├── maps/       # rbtc_room.yaml / .pgm
├── launch/     # sim, bringup, slam, localization, navigation, mission, ekf
├── scripts/    # mission_node.py
├── tests/      # test_waypoints.py
├── docs/       # informes A, B y C
├── package.xml
└── CMakeLists.txt
```

El paquete `dif_bot` es **propio del equipo**, no es material de la cátedra: se generó aquí y compila con `colcon build`.

## Cómo compilar

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Simulación

```bash
ros2 launch dif_bot sim.launch.py                                  # DiffDrive de Gazebo, sin EKF
ros2 launch dif_bot sim.launch.py use_ros2_control:=true           # ros2_control
ros2 launch dif_bot sim.launch.py use_ekf:=true                    # con EKF
ros2 launch dif_bot sim.launch.py headless:=true                   # sin ventana de Gazebo (requiere display para sensores Ogre)
```

Argumentos: `use_ros2_control` (false), `use_ekf` (false), `world` (`rbtc_room.sdf`) y `headless` (false).

**Nota de entorno:** el mundo usa render Ogre v1 (`render_engine ogre`) porque Ogre2 falla en WSL sin GPU (crash al spawnear el robot). Con headless:=true se ejecuta solo el servidor (-s). Los sensores Ogre conservan el acceso al display de WSLg; no se fuerza EGL. Ver `src/dif_bot/docs/informe_A.md`, sección 1.2.

## Práctica 1: control y odometría (persona A)

Dos alternativas para mover las ruedas:

- **DiffDrive de Gazebo** (`use_ros2_control:=false`, defecto): `/cmd_vel` -> `/odom` a 30 Hz y `/tf` odom -> base_link. Lo usan SLAM, AMCL y Nav2 de las prácticas de persona B y C.
- **ros2_control** (`use_ros2_control:=true`): hardware `ign_ros2_control` y `diff_drive_controller`, configurado en `config/controllers.yaml`. El launch **no carga controladores**; hay que cargarlos a mano con la simulación en marcha:

```bash
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active diff_drive_controller
```

Con ros2_control la odometría está en `/diff_drive_controller/odom` y el comando va a `/diff_drive_controller/cmd_vel_unstamped`; `/odom` y `/cmd_vel` no se bridgean en ese modo.

## Práctica 2: EKF (persona A)

Fusión de `/odom` (o `/diff_drive_controller/odom`) y `/imu` con `robot_localization` (`ekf_node`, 2D). Configuración en `src/dif_bot/config/ekf.yaml`, launch en `src/dif_bot/launch/ekf.launch.py`. La salida del EKF va a `/odom_filtered`.

```bash
# Terminal 1: simulación (sin ros2_control)
ros2 launch dif_bot sim.launch.py use_ekf:=true
# Terminal 2: EKF (con IMU por defecto)
ros2 launch dif_bot ekf.launch.py
# Sin IMU, para comparar
ros2 launch dif_bot ekf.launch.py use_imu:=false
```

Con ros2_control:

```bash
ros2 launch dif_bot sim.launch.py use_ros2_control:=true use_ekf:=true
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active diff_drive_controller
ros2 launch dif_bot ekf.launch.py odom_topic:=/diff_drive_controller/odom
```

**Quién publica cada TF:**

- Con `use_ekf:=true`: el EKF publica `odom -> base_link`; la odometría de rueda no lo publica.
- `map -> odom`: lo publica SLAM o AMCL. `ekf.launch.py` trae por defecto un `map -> odom` estático provisional (`launch_static_map_odom:=true`). **Con SLAM o AMCL hay que usar `launch_static_map_odom:=false`**, o habrá dos publicadores de `map -> odom`.

Comprobación rápida: `ros2 topic hz /odom_filtered` y `ros2 run tf2_ros tf2_echo map base_link`. Detalles en `src/dif_bot/docs/informe_A.md`, sección 2.

## SLAM y AMCL (persona B)

Con la simulación en marcha (`sim.launch.py`, sin EKF):

```bash
# Mapeo con SLAM
ros2 launch dif_bot slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # reducir velocidad antes de mover
# Guardar el mapa desde la raíz del repo
ros2 run nav2_map_server map_saver_cli -f src/dif_bot/maps/rbtc_room --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
# Detener SLAM antes de AMCL, luego localización
ros2 launch dif_bot localization.launch.py map:=$HOME/proyecto_RBTC/src/dif_bot/maps/rbtc_room.yaml
```

RViz: Fixed Frame `map`; agregar Map (`/map`), LaserScan (`/scan`), TF y RobotModel. Con AMCL, dar 2D Pose Estimate y observar `/amcl_pose` y `/particle_cloud`. Config: `src/dif_bot/config/rbtc.rviz`.

```bash
ros2 run rviz2 rviz2 -d src/dif_bot/config/rbtc.rviz --ros-args -p use_sim_time:=true
```

TF: Gazebo publica `odom -> base_link`; SLAM o AMCL publica `map -> odom`. Nunca usar SLAM y AMCL a la vez.

## Navegación y misión (persona C)

```bash
./scripts/humble.sh ros2 launch dif_bot navigation.launch.py    # Nav2 (NavFn + DWB), sobre robot y AMCL ya activos
./scripts/humble.sh ros2 launch dif_bot mission.launch.py
./scripts/humble.sh ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

Waypoints en `src/dif_bot/config/waypoints.yaml`. `scripts/humble.sh` abre un entorno limpio de Humble; sin argumentos inicia el bringup completo (`bringup.launch.py`: simulación, localización, navegación y misión). Antes cerrar los lanzamientos existentes.

## Inicio completo desde un clon nuevo

Con ROS 2 Humble y las dependencias anteriores instaladas:

~~~bash
git clone https://github.com/Daniela6666/proyecto_RBTC.git
cd proyecto_RBTC
./scripts/humble.sh colcon build --symlink-install
./scripts/humble.sh
~~~

El ultimo comando inicia robot, EKF, mapa guardado, AMCL, Nav2 y nodo de mision. El EKF publica odom -> base_link y AMCL map -> odom; no se lanza la transformacion estatica provisional de la practica 2. Para comparar sin EKF: ./scripts/humble.sh ros2 launch dif_bot bringup.launch.py use_ekf:=false.

En otra terminal, desde el repositorio:

~~~bash
./scripts/humble.sh ros2 run rviz2 rviz2 -d src/dif_bot/config/rbtc.rviz --ros-args -p use_sim_time:=true
~~~

En RViz, usar 2D Pose Estimate cerca de (0,0), apuntando hacia +X, y verificar que el laser coincida con el mapa. Se pueden enviar metas con Nav2 Goal. Iniciar la ruta solo despues de localizar el robot:

~~~bash
./scripts/humble.sh ros2 service call /mission/start std_srvs/srv/Trigger '{}'
./scripts/humble.sh ros2 topic echo /mission/status
# Cancelacion, desde otra terminal si es necesaria:
./scripts/humble.sh ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
~~~

La respuesta accepted confirma el inicio; el resultado se publica en /mission/status como succeeded, failed, timeout o canceled. Un segundo inicio mientras corre la mision se rechaza. Los cuatro destinos estan en src/dif_bot/config/waypoints.yaml.

Para ejecutar sin ventana de Gazebo: ./scripts/humble.sh ros2 launch dif_bot bringup.launch.py headless:=true. En WSLg debe existir DISPLAY; si Ogre falla, usar LIBGL_ALWAYS_SOFTWARE=1 en el entorno de simulacion. No lanzar dos bringup en el mismo dominio ROS.

Las pruebas reproducibles y sus datos estan en [validation/](validation/README.md). El mapa de entrega es el recorrido lento a 0.2 m/s, con resolucion 0.05 m/celda.

## Flujo de trabajo con ramas

- `main`: rama estable. Solo se integra por Pull Request.
- `persona-a`, `persona-b`, `persona-c`: ramas de trabajo individual.
- Todos los cambios entran a `main` mediante PRs.

## Estado

- Simulación, SLAM, AMCL, Nav2 y misión: implementados (ramas de persona B y C).
- Práctica 1 y 2 (persona A): implementadas en este merge sobre el robot de persona B. Las medidas de la Práctica 1 (`src/dif_bot/docs/informe_A.md`, sección 1) se repitieron con el robot actual el 2026-11-22; las de la Práctica 2 (sección 2) siguen siendo del robot anterior y hay que repetirlas.
- Pendiente: exploración completa y ensayos P3/P4 (B), video y informe final.
