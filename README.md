# Proyecto RBTC
Robot diferencial creado desde cero. ROS 2 Humble, Gazebo Fortress, ros_gz. Mundo local rbtc_room de 8 x 8 metros, tres obstaculos.

## Equipo y ramas
A: Daniela Molina (P1/P2). B: Tomas Alvarez (P3/P4). C: por definir (P5/mision). Ramas individuales; main se integra mediante PR.

## Dependencias
    sudo apt-get update
    sudo apt-get install ros-humble-ros-gz ros-humble-robot-state-publisher ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-teleop-twist-keyboard python3-colcon-common-extensions

## Compilar y ejecutar
    source /opt/ros/humble/setup.bash
    cd ~/proyecto_RBTC
    colcon build --symlink-install
    source install/setup.bash
    ros2 launch dif_bot sim.launch.py
No cargar Iron junto con Humble. El launch usa renderizado por software para evitar el fallo OGRE observado en WSL.
En otra terminal con el mismo entorno:
    ros2 launch dif_bot slam.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
Reducir velocidad antes de mover. Guardar desde la raiz del repo:
    ros2 run nav2_map_server map_saver_cli -f src/dif_bot/maps/rbtc_room --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
Detener SLAM antes de iniciar AMCL:
    ros2 launch dif_bot localization.launch.py map:=$HOME/proyecto_RBTC/src/dif_bot/maps/rbtc_room.yaml
    ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
RViz: Fixed Frame map; agregar Map y LaserScan; usar 2D Pose Estimate y mover despacio.

## Estado
Compilacion verificada, LiDAR recibido, mapa preliminar guardado a 0.05 m/celda, AMCL activo y publicando pose durante movimiento.
Pendientes: explorar zonas ocultas y ensayos P3/P4; EKF y alternativa ros2_control (A); video de la mision (C); video e informe final. El launch actual NO implementa use_ros2_control.
Ver src/dif_bot/docs/informe_B.md.

## Parte C implementada
Nav2 (NavFn + DWB), configuracion RViz, waypoints YAML, nodo de mision con servicios start/cancel, estado JSON y launch integrado. Mision validada: 4/4 objetivos, succeeded. Ver src/dif_bot/docs/informe_C.md y evidencias.
Entorno limpio: ./scripts/humble.sh [comando y argumentos]. Sin argumentos inicia bringup completo; antes cerrar los lanzamientos existentes.
Para navegar sobre robot y AMCL ya activos:
    ./scripts/humble.sh ros2 launch dif_bot navigation.launch.py
    ./scripts/humble.sh ros2 launch dif_bot mission.launch.py
    ./scripts/humble.sh ros2 service call /mission/start std_srvs/srv/Trigger '{}'
RViz:
    ./scripts/humble.sh ros2 run rviz2 rviz2 -d src/dif_bot/config/rbtc.rviz --ros-args -p use_sim_time:=true
No se grabo video ni se genero PDF final; siguen pendientes ensayos P3/P4 y la integracion de P1/P2.
