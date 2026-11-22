# Robótica Móvil: proyecto final

Simulación de un robot diferencial (`dif_bot`) en Gazebo Fortress con ROS 2 Humble, usando el puente `ros_gz`. Incluye control y odometría (Práctica 1) y fusión sensorial con EKF (Práctica 2).

## Versiones

- ROS 2 Humble
- Gazebo Fortress (Ignition)
- ros_gz

## Mundo

Mundo `empty_world` (`src/dif_bot/worlds/empty_world.sdf`): suelo (`ground_plane`), luz `sun` y 3 cajas estáticas como obstáculos para el lidar.

## Dependencias

```bash
sudo apt-get update && sudo apt-get install -y ros-humble-ign-ros2-control ros-humble-plotjuggler-ros ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-robot-localization ros-humble-xacro ros-humble-rqt-plot ros-humble-tf2-tools
sudo apt-get install -y ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-ros-gz
```

## Estructura del repo

```
src/dif_bot/
├── urdf/       # robot.urdf.xacro, gz_control.xacro, ros2_control.xacro
├── worlds/     # empty_world.sdf
├── rviz/       # dif_bot.rviz
├── config/     # controllers.yaml
├── maps/       # mapas
├── launch/     # sim.launch.py
├── docs/       # informes
├── package.xml
└── CMakeLists.txt
```

El paquete `dif_bot` (robot diferencial, mundo y launch) es **propio del equipo**, no es material de la cátedra: se generó aquí y compila con `colcon build`.

## Cómo compilar

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Cómo lanzar

```bash
ros2 launch dif_bot sim.launch.py use_ros2_control:=false
ros2 launch dif_bot sim.launch.py use_ros2_control:=true
```

Argumentos: `use_ros2_control` (false por defecto), `world` (`empty_world.sdf` por defecto) y `headless` (false por defecto; `true` arranca el servidor sin ventana con render EGL).

Con `use_ros2_control:=true` el launch **no carga controladores**. Hay que cargarlos a mano con el launch ya en marcha:

```bash
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active diff_drive_controller
```

Con ros2_control la odometría NO está en `/odom` y el comando NO va a `/cmd_vel`; están en `/diff_drive_controller/odom` y `/diff_drive_controller/cmd_vel_unstamped`.

**Nota de entorno:** el mundo usa render Ogre v1 (`render_engine ogre`) porque Ogre2 falla en WSL sin GPU. Ver `src/dif_bot/docs/informe_A.md`, sección 1.2.

## Práctica 2: EKF

Fusión de `/odom` (o `/diff_drive_controller/odom`) y `/imu` con `robot_localization` (`ekf_node`, 2D). Configuración en `src/dif_bot/config/ekf.yaml`, launch en `src/dif_bot/launch/ekf.launch.py`.

Con `use_ekf:=true` el EKF publica el TF `odom -> base_link` y la odometría de rueda no lo publica (evita doble TF). La salida del EKF va a `/odom_filtered`.

```bash
# Terminal 1: simulación (sin ros2_control)
ros2 launch dif_bot sim.launch.py use_ros2_control:=false use_ekf:=true
# Terminal 2: EKF (con IMU por defecto)
ros2 launch dif_bot ekf.launch.py
# Sin IMU, para comparar
ros2 launch dif_bot ekf.launch.py use_imu:=false
```

Con ros2_control (los controladores se cargan a mano, como en la Práctica 1):

```bash
ros2 launch dif_bot sim.launch.py use_ros2_control:=true use_ekf:=true
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active diff_drive_controller
ros2 launch dif_bot ekf.launch.py odom_topic:=/diff_drive_controller/odom
```

Comprobación rápida: `ros2 topic hz /odom_filtered` y `ros2 run tf2_ros tf2_echo map base_link`. Detalles y resultados en `src/dif_bot/docs/informe_A.md`, sección 2.

## Flujo de trabajo con ramas

- `main`: rama estable. Solo se integra por Pull Request.
- `persona-a`, `persona-b`, `persona-c`: ramas de trabajo individual.
- Todos los cambios entran a `main` mediante PRs.

## Reparto

| Persona | Responsable | Prácticas |
|---|---|---|
| A | Daniela Molina | Prácticas 1 y 2 |
| B | por definir | por definir |
| C | por definir | por definir |
