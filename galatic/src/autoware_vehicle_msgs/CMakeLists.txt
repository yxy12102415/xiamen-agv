cmake_minimum_required(VERSION 3.5)
project(autoware_vehicle_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlModeReport.msg"
  "msg/Engage.msg"
  "msg/GearCommand.msg"
  "msg/GearReport.msg"
  "msg/HazardLightsCommand.msg"
  "msg/HazardLightsReport.msg"
  "msg/SteeringReport.msg"
  "msg/TurnIndicatorsCommand.msg"
  "msg/TurnIndicatorsReport.msg"
  "msg/VelocityReport.msg"
  "srv/ControlModeCommand.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
