cmake_minimum_required(VERSION 3.8)
project(agv_gazebo)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_gz_interfaces QUIET)
if(ros_gz_interfaces_FOUND)
  set(GAZEBO_ROS_INTERFACES_TARGET ros_gz_interfaces)
else()
  find_package(ros_ign_interfaces REQUIRED)
  set(GAZEBO_ROS_INTERFACES_TARGET ros_ign_interfaces)
  add_compile_definitions(AGV_USE_ROS_IGN_INTERFACES)
endif()
find_package(sensor_msgs REQUIRED)

foreach(IGN_TRANSPORT_VERSION 11 10)
  find_package(ignition-transport${IGN_TRANSPORT_VERSION} QUIET)
  if(TARGET ignition-transport${IGN_TRANSPORT_VERSION}::ignition-transport${IGN_TRANSPORT_VERSION})
    set(IGNITION_TRANSPORT_TARGET
      ignition-transport${IGN_TRANSPORT_VERSION}::ignition-transport${IGN_TRANSPORT_VERSION}
    )
    break()
  endif()
endforeach()

foreach(IGN_MSGS_VERSION 8 7)
  find_package(ignition-msgs${IGN_MSGS_VERSION} QUIET)
  if(TARGET ignition-msgs${IGN_MSGS_VERSION}::ignition-msgs${IGN_MSGS_VERSION})
    set(IGNITION_MSGS_TARGET ignition-msgs${IGN_MSGS_VERSION}::ignition-msgs${IGN_MSGS_VERSION})
    break()
  endif()
endforeach()

if(IGNITION_TRANSPORT_TARGET AND IGNITION_MSGS_TARGET)
  add_compile_definitions(AGV_USE_IGN_TRANSPORT_FOR_POSE)
endif()

add_executable(kinematic_state_to_gazebo_node
  src/kinematic_state_to_gazebo_node.cpp
)
target_compile_features(kinematic_state_to_gazebo_node PUBLIC cxx_std_17)
ament_target_dependencies(kinematic_state_to_gazebo_node
  geometry_msgs
  nav_msgs
  rclcpp
  ${GAZEBO_ROS_INTERFACES_TARGET}
)

if(IGNITION_TRANSPORT_TARGET AND IGNITION_MSGS_TARGET)
  target_link_libraries(kinematic_state_to_gazebo_node
    ${IGNITION_TRANSPORT_TARGET}
    ${IGNITION_MSGS_TARGET}
  )
endif()

add_executable(gnss_imu_simulator_node
  src/gnss_imu_simulator_node.cpp
)
target_compile_features(gnss_imu_simulator_node PUBLIC cxx_std_17)
ament_target_dependencies(gnss_imu_simulator_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

add_executable(dynamic_obstacle_simulator_node
  src/dynamic_obstacle_simulator_node.cpp
)
target_compile_features(dynamic_obstacle_simulator_node PUBLIC cxx_std_17)
ament_target_dependencies(dynamic_obstacle_simulator_node
  geometry_msgs
  rclcpp
  ${GAZEBO_ROS_INTERFACES_TARGET}
)

if(IGNITION_TRANSPORT_TARGET AND IGNITION_MSGS_TARGET)
  target_link_libraries(dynamic_obstacle_simulator_node
    ${IGNITION_TRANSPORT_TARGET}
    ${IGNITION_MSGS_TARGET}
  )
endif()

add_executable(pointcloud_frame_republisher_node
  src/pointcloud_frame_republisher_node.cpp
)
target_compile_features(pointcloud_frame_republisher_node PUBLIC cxx_std_17)
ament_target_dependencies(pointcloud_frame_republisher_node
  rclcpp
  sensor_msgs
)

add_executable(wheel_steering_visualizer_node
  src/wheel_steering_visualizer_node.cpp
)
target_compile_features(wheel_steering_visualizer_node PUBLIC cxx_std_17)
ament_target_dependencies(wheel_steering_visualizer_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  ${GAZEBO_ROS_INTERFACES_TARGET}
)

if(IGNITION_TRANSPORT_TARGET AND IGNITION_MSGS_TARGET)
  target_link_libraries(wheel_steering_visualizer_node
    ${IGNITION_TRANSPORT_TARGET}
    ${IGNITION_MSGS_TARGET}
  )
endif()

set(AGV_GAZEBO_TARGETS
  kinematic_state_to_gazebo_node
  gnss_imu_simulator_node
  dynamic_obstacle_simulator_node
  pointcloud_frame_republisher_node
  wheel_steering_visualizer_node
)

if(IGNITION_TRANSPORT_TARGET AND IGNITION_MSGS_TARGET)
  add_executable(ign_pointcloud_bridge_node
    src/ign_pointcloud_bridge_node.cpp
  )
  target_compile_features(ign_pointcloud_bridge_node PUBLIC cxx_std_17)
  ament_target_dependencies(ign_pointcloud_bridge_node
    rclcpp
    sensor_msgs
  )
  target_link_libraries(ign_pointcloud_bridge_node
    ${IGNITION_TRANSPORT_TARGET}
    ${IGNITION_MSGS_TARGET}
  )
  list(APPEND AGV_GAZEBO_TARGETS ign_pointcloud_bridge_node)
endif()

install(
  DIRECTORY config launch models scripts worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/remote_drive_camera_viewer.py
  RENAME remote_drive_camera_viewer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    ${AGV_GAZEBO_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
