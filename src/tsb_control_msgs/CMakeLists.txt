cmake_minimum_required(VERSION 3.8)
project(tsb_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AllocationFeedback.msg
  msg/PathStatus.msg
  msg/PidDebug.msg
  msg/PidTerms.msg
  msg/Reference.msg
  msg/VesselState.msg
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
