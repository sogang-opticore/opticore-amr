cmake_minimum_required(VERSION 3.8)
project(amr_bringup)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  worlds
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    src/dynamic_obstacle_mover.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()
