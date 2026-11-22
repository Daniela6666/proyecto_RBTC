# Robótica Móvil: proyecto final

Simulación de un robot diferencial (`dif_bot`) en Gazebo Fortress con ROS 2 Humble, usando el puente `ros_gz`. Incluye control y odometría (Práctica 1) y fusión sensorial con EKF (Práctica 2).

## Versiones

- ROS 2 Humble
- Gazebo Fortress (Ignition)
- ros_gz

## Mundo

Pendiente de definir.

## Dependencias

```bash
sudo apt-get update && sudo apt-get install -y ros-humble-ign-ros2-control ros-humble-plotjuggler-ros ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-robot-localization ros-humble-xacro ros-humble-rqt-plot ros-humble-tf2-tools
sudo apt-get install -y ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-ros-gz
```

## Estructura del repo

```
src/dif_bot/
├── config/     # parámetros (YAML)
├── maps/       # mapas
├── launch/     # launch files
├── docs/       # informes
└── README_PAQUETE.md
```

El paquete `dif_bot` real (urdf/, worlds/, package.xml, CMakeLists.txt, launch/sim.launch.py) se copia en `src/dif_bot/` según `README_PAQUETE.md`. Sin ese paquete no compila ni lanza.

## Cómo compilar

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Cómo lanzar

Requiere el paquete dif_bot real en `src/dif_bot/`.

```bash
ros2 launch dif_bot sim.launch.py use_ros2_control:=false
ros2 launch dif_bot sim.launch.py use_ros2_control:=true
```

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
