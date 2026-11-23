# P5 y proyecto final: navegacion y mision
Responsable de implementacion: Tomas Alvarez (partes B y C).

## Configuracion
NavFn (nav2_navfn_planner/NavfnPlanner) con Dijkstra, DWB como controlador local, velocidad maxima lineal 0.18 m/s y angular 0.5 rad/s. Radio conservador 0.30 m; inflacion 0.45 m. Ambos costmaps usan /scan y use_sim_time.
El arbol instalado navigate_to_pose_w_replanning_and_recovery.xml replantea la ruta y permite limpieza de costmaps, giro de 1.57 rad, espera de 5 s y retroceso de 0.30 m a 0.05 m/s. Son comportamientos configurados: no afirmar que todos se ejecutaron sin evidencia del ensayo.

## Mision
scripts/mission_node.py usa nav2_simple_commander. Lee config/waypoints.yaml al iniciar cada mision, valida coordenadas finitas y frame map. Ejecuta cuatro objetivos cercanos al centro del mapa preliminar. No inicia automaticamente al arrancar.
/mission/start (std_srvs/srv/Trigger) acepta la solicitud, no certifica que la mision haya terminado. /mission/status (std_msgs/msg/String, JSON, transient local) informa idle, waiting_for_nav2, navigating, waypoint_reached, succeeded, canceled, timeout o failed.
/mission/cancel solicita cancelacion. Se rechazan solicitudes simultaneas. El trabajador de navegacion tiene un ejecutor distinto del servicio para mantenerlo disponible. Tiempo limite por objetivo: 300 s de reloj real; servidor Nav2: 30 s.

## Ejecucion
En terminal Humble limpia, despues de compilar:
    source /opt/ros/humble/setup.bash
    source ~/proyecto_RBTC/install/local_setup.bash
    ros2 launch dif_bot bringup.launch.py
No ejecutar bringup encima de una simulacion ya activa. El launch incluye robot, mapa, AMCL, Nav2 y mision. No incluye EKF (pendiente de A).
En RViz: Fixed Frame map; dar 2D Pose Estimate antes de iniciar. Para metas manuales usar Nav2 Goal.
    ros2 service call /mission/start std_srvs/srv/Trigger '{}'
    ros2 topic echo /mission/status --qos-durability transient_local
    ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
Si robot y AMCL ya estan activos, lanzar solo navigation.launch.py y mission.launch.py.

## Evidencias
Nav2 llego a Managed nodes are active. Se verifico aceptacion de inicio y rechazo de segunda solicitud con Mission already running. El recorrido completo y cancelacion efectiva se documentan tras concluir las pruebas.
Para video: mostrar Gazebo, RViz (mapa, laser, plan y pose) y terminal con /mission/status. Mostrar inicio por servicio, cuatro objetivos y resultado final. No existe aun video final.

## Pruebas de servicios verificadas
Inicio aceptado; segunda solicitud rechazada con Mission already running. Cancelacion aceptada y estado canceled observado durante el primer objetivo. Se corrigio conflicto de ejecutores y se recompilo con Humble aislado (el setup antiguo arrastraba Iron).
Usar scripts/humble.sh para ejecutar comandos en entorno limpio. Sin argumentos lanza bringup; no usar mientras otra simulacion este activa.

## Resultado observado (22 noviembre 2026)
Mision final: 4/4 objetivos alcanzados, estado succeeded con completed=4. Evidencia: docs/evidencias/mission_states.log (incluye intentos y cancelaciones anteriores).
Se verificaron inicio, rechazo de inicio simultaneo y cancelacion efectiva. Pruebas del lector YAML: 2 tests correctos, incluyendo casos de formato, frame y numeros invalidos.
Durante la depuracion hubo fallos de progreso y recuperaciones. Se igualo FollowPath.xy_goal_tolerance con el goal checker a 0.15 m y required_movement_radius a 0.10 m. El reinicio limpio y la reduccion de carga grafica permitieron completar el recorrido.
La GUI de Gazebo se pauso temporalmente durante la prueba final; fisica, sensores y Nav2 continuaron activos. Se reanudo al terminar. No confundir esta prueba con un video final ni con una validacion completa del mapa: sigue habiendo zonas desconocidas cerca de algunos waypoints y Nav2 utilizo tambien los sensores en vivo.

## Guion de video pendiente
1. Mostrar mapa y robot en Gazebo/RViz.
2. Mostrar Nav2 activo y pose inicial alineada.
3. Iniciar con /mission/start y observar /mission/status.
4. Mostrar objetivos alcanzados y succeeded, completed=4.
5. Explicar NavFn/DWB, servicio de inicio y cancelacion.
Grabar una nueva ejecucion con GUI activa; esta sesion no produjo archivo de video.
