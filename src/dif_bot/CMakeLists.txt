cmake_minimum_required(VERSION 3.8)
project(dif_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

# Persona C: nodos Python de navegación y misión
install(PROGRAMS
  scripts/mission_node.py
  scripts/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
