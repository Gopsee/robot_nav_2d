cmake_minimum_required(VERSION 3.8)
project(trajectory_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_controller REQUIRED)  # for FollowPath.action

# -------------------------------------------------------------
# Library (for node + tests)
# -------------------------------------------------------------
add_library(trajectory_generator_lib
  src/trajectory_generator.cpp
)

target_include_directories(trajectory_generator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(trajectory_generator_lib
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  trajectory_controller
)

# -------------------------------------------------------------
# Executable Node
# -------------------------------------------------------------
add_executable(trajectory_generator_node
  src/trajectory_generator_node.cpp
)

target_link_libraries(trajectory_generator_node
  trajectory_generator_lib
)

ament_target_dependencies(trajectory_generator_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  trajectory_controller
)

# -------------------------------------------------------------
# Install
# -------------------------------------------------------------
install(TARGETS
  trajectory_generator_node
  trajectory_generator_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# -------------------------------------------------------------
# Unit Tests
# -------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_trajectory_generator
    test/test_trajectory_generator.cpp
  )

  target_link_libraries(test_trajectory_generator
    trajectory_generator_lib
  )

  ament_target_dependencies(test_trajectory_generator
    rclcpp
    rclcpp_action
    geometry_msgs
    nav_msgs
  )
endif()

# -------------------------------------------------------------
# Export
# -------------------------------------------------------------
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  trajectory_controller
)
ament_export_include_directories(include)

ament_package()
