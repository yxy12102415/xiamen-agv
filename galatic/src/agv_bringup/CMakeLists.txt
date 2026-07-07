cmake_minimum_required(VERSION 3.8)
project(agv_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/agv_set_route
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
