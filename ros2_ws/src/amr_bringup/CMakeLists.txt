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

ament_package()
