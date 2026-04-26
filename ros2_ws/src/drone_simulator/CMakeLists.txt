cmake_minimum_required(VERSION 3.8)
project(drone_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Drone kinematics (replaces simple_localization_node)
add_executable(drone_kinematics_node src/drone_kinematics_node.cpp)
ament_target_dependencies(drone_kinematics_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

# Drone global planner (3D-aware, adds cruise altitude to waypoints)
add_executable(drone_global_planner_node src/drone_global_planner_node.cpp)
ament_target_dependencies(drone_global_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Drone lidar (RS-16 sim with dynamic altitude from odometry)
add_executable(drone_lidar_node src/drone_lidar_node.cpp)
ament_target_dependencies(drone_lidar_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  drone_kinematics_node
  drone_global_planner_node
  drone_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
