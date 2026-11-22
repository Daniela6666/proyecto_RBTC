# Pruebas reproducibles B/C

Ejecutar desde la raiz del workspace, en Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress.
Dependencias adicionales del evaluador: python3-numpy, python3-pil y python3-yaml.
El evaluador utiliza ROS_DOMAIN_ID=43 e IGN_PARTITION=rbtc_batch para aislar las pruebas.
Ejecutar los casos secuencialmente; cada caso inicia y detiene sus propios procesos.
En WSL, los sensores Ogre requieren WSLg (DISPLAY=:0), aunque no se abra la GUI.

~~~bash
./scripts/humble.sh colcon build --packages-select dif_bot
./scripts/humble.sh python3 scripts/validate_bc.py mapping_slow
# Instalar el mapa actualizado antes de probar localizacion/navegacion.
./scripts/humble.sh colcon build --packages-select dif_bot
./scripts/humble.sh python3 scripts/validate_bc.py mapping_fast
for c in amcl_correct_low amcl_wrong_low amcl_correct_high amcl_wrong_high; do
  ./scripts/humble.sh python3 scripts/validate_bc.py "$c"
done
./scripts/humble.sh python3 scripts/validate_navigation.py
./scripts/humble.sh python3 -m unittest discover -s src/dif_bot/tests -v
~~~

- SLAM: mismo recorrido, velocidades lineales limite 0.2 y 0.5 m/s, ruido laser sigma=0.01 m. Se guarda cada mapa; el lento se instala como mapa del proyecto.
- AMCL: pose inicial correcta o desplazada +1 m en x/y y +0.6 rad; ruido sigma=0.01/0.05 m. Se gira y recorre el mismo circuito corto. La referencia es la pose del modelo en Gazebo, recibida de /world/rbtc_room/dynamic_pose/info; no es la odometria de ruedas.
- Convergencia: cinco muestras seguidas con error de posicion <0.2 m y de orientacion <0.15 rad. null significa que no se alcanzo ese criterio. Los errores finales son medianas de las diez ultimas muestras. Las poses se comparan al recibir AMCL con la referencia mas reciente, sin interpolacion temporal.
- Mapa: cobertura conocida dentro del espacio libre teorico del mundo; distancia de celdas ocupadas a paredes/obstaculos teoricos (media y percentil 95). Es una comprobacion geometrica, no una medida absoluta de exactitud del SLAM.
- Navegacion: nodos lifecycle activos, meta individual por la misma accion usada por RViz, inicio de mision, rechazo de inicio duplicado, cancelacion y posterior mision completa.

Los JSON/CSV/PGM son datos de una ejecucion por condicion, no un estudio estadistico. Los registros de procesos se escriben en /tmp/rbtc_*.log. Los ensayos del modelo anterior se conservaron localmente en una copia de diagnostico y no se incluyen como resultados finales.
