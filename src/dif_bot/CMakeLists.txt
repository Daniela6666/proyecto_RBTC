cmake_minimum_required(VERSION 3.8)
project(dif_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds launch config maps rviz docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/mission_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
