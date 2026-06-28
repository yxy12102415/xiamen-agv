cmake_minimum_required(VERSION 3.8)
project(agv_maps)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
