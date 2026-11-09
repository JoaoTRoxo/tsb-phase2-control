cmake_minimum_required(VERSION 3.8)
project(tsb_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tsb_control_msgs REQUIRED)

include_directories(include)

set(NODES sensor_processing_node pid_node allocation_node path_following_node)
foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  ament_target_dependencies(${node}
    geometry_msgs nav_msgs rcl_interfaces rclcpp sensor_msgs std_msgs tsb_control_msgs)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(PROGRAMS
  scripts/reference_sequence.py
  scripts/send_waypoints.py
  scripts/plot_run.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # The repository has no per-file licence headers, so that one linter is skipped.
  set(ament_cmake_copyright_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name test_pid test_allocation test_filters test_los test_closed_loop)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
  endforeach()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_pipeline.py TIMEOUT 60)
endif()

ament_package()
