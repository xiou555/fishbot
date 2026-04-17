cmake_minimum_required(VERSION 3.8)
project(fishbot_four_wheel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(four_wheel_commander src/four_wheel_commander.cpp)
ament_target_dependencies(four_wheel_commander
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_executable(motion_test_node test/motion_test_node.cpp)
ament_target_dependencies(motion_test_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_executable(four_wheel_closed_loop_odom src/four_wheel_closed_loop_odom.cpp)
ament_target_dependencies(four_wheel_closed_loop_odom
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)


install(TARGETS
  four_wheel_commander
  motion_test_node
  four_wheel_closed_loop_odom
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
