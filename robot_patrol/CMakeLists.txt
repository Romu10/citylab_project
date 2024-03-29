cmake_minimum_required(VERSION 3.5)
project(robot_patrol)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(robot_patrol_node src/patrol.cpp)
ament_target_dependencies(robot_patrol_node rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs)

add_executable(service_server_node src/direction_service.cpp)
ament_target_dependencies(service_server_node rclcpp geometry_msgs std_msgs custom_interfaces)

add_executable(service_test_node src/test_service.cpp)
ament_target_dependencies(service_test_node rclcpp std_msgs custom_interfaces sensor_msgs)

add_executable(patrolling_node src/patrol_with_service.cpp)
ament_target_dependencies(patrolling_node rclcpp std_msgs custom_interfaces sensor_msgs geometry_msgs)

add_executable(go_to_pose_node src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_node rclcpp std_msgs custom_interfaces sensor_msgs geometry_msgs nav_msgs rclcpp_action)

install(TARGETS
   robot_patrol_node
   service_server_node
   service_test_node
   patrolling_node
   go_to_pose_node
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
