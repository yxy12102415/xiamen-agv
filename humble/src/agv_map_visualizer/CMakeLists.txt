cmake_minimum_required(VERSION 3.8)
project(agv_map_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autoware_lanelet2_utils QUIET)
find_package(autoware_map_msgs QUIET)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(AGV_MAP_VISUALIZER_TARGETS
  agv_vehicle_marker_publisher
  static_obstacle_marker_publisher
  lidar_point_marker_publisher
)

if(autoware_lanelet2_utils_FOUND AND autoware_map_msgs_FOUND)
  add_executable(lanelet2_marker_publisher src/lanelet2_marker_publisher.cpp)
  target_compile_features(lanelet2_marker_publisher PUBLIC cxx_std_17)
  ament_target_dependencies(lanelet2_marker_publisher
    autoware_lanelet2_utils
    autoware_map_msgs
    rclcpp
    visualization_msgs
  )
  list(APPEND AGV_MAP_VISUALIZER_TARGETS lanelet2_marker_publisher)
else()
  message(STATUS "Skipping lanelet2_marker_publisher because Autoware map dependencies were not found")
endif()

add_executable(agv_vehicle_marker_publisher src/agv_vehicle_marker_publisher.cpp)
target_compile_features(agv_vehicle_marker_publisher PUBLIC cxx_std_17)
ament_target_dependencies(agv_vehicle_marker_publisher
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  visualization_msgs
)

add_executable(static_obstacle_marker_publisher src/static_obstacle_marker_publisher.cpp)
target_compile_features(static_obstacle_marker_publisher PUBLIC cxx_std_17)
ament_target_dependencies(static_obstacle_marker_publisher
  geometry_msgs
  rclcpp
  visualization_msgs
)

add_executable(lidar_point_marker_publisher src/lidar_point_marker_publisher.cpp)
target_compile_features(lidar_point_marker_publisher PUBLIC cxx_std_17)
ament_target_dependencies(lidar_point_marker_publisher
  geometry_msgs
  rclcpp
  sensor_msgs
  visualization_msgs
)

install(TARGETS
  ${AGV_MAP_VISUALIZER_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/osm_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
