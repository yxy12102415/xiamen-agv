cmake_minimum_required(VERSION 3.5)
project(autoware_simple_planning_simulator)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
