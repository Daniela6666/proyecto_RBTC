# Persona B: SLAM y AMCL
Robot diferencial creado desde cero: ROS 2 Humble / Gazebo Fortress. Mundo rbtc_room de 8 x 8 metros con tres obstaculos. Ruedas radio 0.1 m, separacion 0.38 m. LiDAR 360 muestras, 10 Hz; IMU 50 Hz.
Estado: compilacion verificada, simulacion y LiDAR funcionando, mapa preliminar guardado a 0.05 m/celda, AMCL activo y publicando pose. Faltan exploracion completa y ensayos comparativos.
Compilar: colcon build --symlink-install; source install/setup.bash
Simular: ros2 launch dif_bot sim.launch.py
Mapear en otra terminal: ros2 launch dif_bot slam.launch.py
RViz: ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
Fixed Frame map; agregar Map (/map, transient local), LaserScan (/scan), TF y RobotModel.
Teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard. Reducir velocidad antes de mover a aproximadamente 0.15 m/s y 0.3 rad/s.
Guardar desde la raiz del repo: ros2 run nav2_map_server map_saver_cli -f src/dif_bot/maps/rbtc_room --ros-args -p use_sim_time:=true
Detener SLAM antes de AMCL: ros2 launch dif_bot localization.launch.py map:=$HOME/proyecto_RBTC/src/dif_bot/maps/rbtc_room.yaml
Dar 2D Pose Estimate en RViz; observar /amcl_pose y /particle_cloud.
TF: Gazebo publica odom -> base_link; SLAM o AMCL publica map -> odom. Nunca usar ambos simultaneamente.
Pruebas P3 pendientes: verificar resolucion 0.05 m/celda del mapa guardado, errores geometricos y efecto de recorrer rapido/lento.
Pruebas P4 pendientes: pose inicial incorrecta y ruido laser stddev 0.01 vs 0.05 m. Medir convergencia contra Gazebo, conservar el mismo mapa.
La alternativa ros2_control y el EKF de persona A siguen pendientes. Nav2 y la mision de persona C estan implementados y probados; ver informe_C.md.

## Verificacion final de esta etapa
Compilacion correcta. Mapa preliminar guardado con resolucion 0.05 m/celda. AMCL active [3], /amcl_pose recibido durante movimiento: x=0.01945 m, y=0.02767 m. Son estimaciones, no errores medidos contra ground truth. Quedan pendientes exploracion completa y ensayos comparativos.
