cmake_minimum_required(VERSION 3.8)
project(agv_vehicle_model)

include(CTest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autoware_adapi_v1_msgs QUIET)
find_package(autoware_control_msgs QUIET)
find_package(autoware_simple_planning_simulator QUIET)
find_package(autoware_vehicle_msgs QUIET)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(agv_standalone_simulator_node
  src/agv_standalone_simulator_node.cpp
)
target_compile_features(agv_standalone_simulator_node PUBLIC cxx_std_17)
ament_target_dependencies(agv_standalone_simulator_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_msgs
  tf2_ros
)

set(AGV_VEHICLE_MODEL_TARGETS agv_standalone_simulator_node)

if(
  autoware_adapi_v1_msgs_FOUND AND
  autoware_control_msgs_FOUND AND
  autoware_simple_planning_simulator_FOUND AND
  autoware_vehicle_msgs_FOUND
)
  find_package(std_msgs REQUIRED)

  add_executable(agv_two_axis_simulator_node
    src/agv_two_axis_simulator_node.cpp
    src/sim_model_delay_steer_vel_two_axis.cpp
  )
  target_compile_features(agv_two_axis_simulator_node PUBLIC cxx_std_17)
  target_include_directories(agv_two_axis_simulator_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(agv_two_axis_simulator_node
    autoware_adapi_v1_msgs
    autoware_control_msgs
    autoware_simple_planning_simulator
    autoware_vehicle_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    tf2
    std_msgs
    tf2_msgs
    tf2_ros
  )
  target_link_libraries(agv_two_axis_simulator_node Eigen3::Eigen)
  list(APPEND AGV_VEHICLE_MODEL_TARGETS agv_two_axis_simulator_node)

  add_executable(agv_keyboard_teleop_node
    src/agv_keyboard_teleop_node.cpp
  )
  target_compile_features(agv_keyboard_teleop_node PUBLIC cxx_std_17)
  ament_target_dependencies(agv_keyboard_teleop_node
    autoware_control_msgs
    rclcpp
  )
  list(APPEND AGV_VEHICLE_MODEL_TARGETS agv_keyboard_teleop_node)
else()
  message(STATUS "Building Galactic standalone agv_two_axis_simulator_node without Autoware message dependencies")
  add_executable(agv_two_axis_simulator_node
    src/agv_standalone_simulator_node.cpp
  )
  target_compile_features(agv_two_axis_simulator_node PUBLIC cxx_std_17)
  ament_target_dependencies(agv_two_axis_simulator_node
    geometry_msgs
    nav_msgs
    rclcpp
    tf2
    tf2_msgs
    tf2_ros
  )
  list(APPEND AGV_VEHICLE_MODEL_TARGETS agv_two_axis_simulator_node)
endif()

if(BUILD_TESTING)
  add_executable(test_ackermann_geometry
    test/test_ackermann_geometry.cpp
  )
  target_compile_features(test_ackermann_geometry PUBLIC cxx_std_17)
  target_include_directories(test_ackermann_geometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  add_test(NAME test_ackermann_geometry COMMAND test_ackermann_geometry)

  if(
    autoware_adapi_v1_msgs_FOUND AND
    autoware_control_msgs_FOUND AND
    autoware_simple_planning_simulator_FOUND AND
    autoware_vehicle_msgs_FOUND
  )
    add_executable(test_vehicle_dynamics_regression
      test/test_vehicle_dynamics_regression.cpp
      src/sim_model_delay_steer_vel_two_axis.cpp
    )
    target_compile_features(test_vehicle_dynamics_regression PUBLIC cxx_std_17)
    target_include_directories(test_vehicle_dynamics_regression PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_vehicle_dynamics_regression
      autoware_simple_planning_simulator
    )
    target_link_libraries(test_vehicle_dynamics_regression Eigen3::Eigen)
    add_test(
      NAME test_vehicle_dynamics_regression
      COMMAND test_vehicle_dynamics_regression
    )
  endif()
endif()

install(
  DIRECTORY include
  DESTINATION .
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    ${AGV_VEHICLE_MODEL_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
