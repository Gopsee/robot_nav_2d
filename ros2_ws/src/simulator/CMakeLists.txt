cmake_minimum_required(VERSION 3.8)
project(simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Simple localization node
add_executable(simple_localization_node src/simple_localization_node.cpp)
ament_target_dependencies(simple_localization_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

# Global planner node
add_executable(global_planner_node src/global_planner_node.cpp)
ament_target_dependencies(global_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Lidar simulation node
add_executable(lidar_node src/lidar_node.cpp)
ament_target_dependencies(lidar_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

# Random goal publisher
add_executable(random_goal_node src/random_goal_node.cpp)
ament_target_dependencies(random_goal_node
  rclcpp
  geometry_msgs
)

install(TARGETS
  simple_localization_node
  global_planner_node
  lidar_node
  random_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
